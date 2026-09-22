# STM day_1 hw2 Report

---

## 1단계: adc_updated_flag 없이 while 내부에서 adc1_buffer[0]을 계속 읽으면 어떤 문제가 생길 수 있는가?

DMA는 CPU와 독립적으로 동시에 동작하면서 buffer에 값을 쓴다.
따라서 flag 없이 아무 때나 읽으면, DMA가 buffer를 갱신하는 도중에
읽게 되어 불완전한(찢어진) 값을 읽을 수 있다.
특히 4채널을 처리할 때, buffer[0]은 새 값이고 buffer[1]은 아직
이전 값인 것처럼 서로 다른 시점의 데이터가 섞일 수 있다.

adc_updated_flag는 DMA가 변환을 완료(buffer를 다 채움)했을 때
콜백에서 세워지므로, flag가 1일 때만 읽으면 항상 완전한 한 세트의
데이터를 안전하게 읽을 수 있다. (데이터 무결성 보장)

(부수적으로: flag를 쓰면 새 데이터가 왔을 때만 처리하므로,
같은 값을 중복 처리하거나 불필요하게 계산하는 낭비도 줄어든다.)

---

## 2단계: PSD 정규화 · 이동평균 필터 · 원본값과 필터값 비교

- 2-1. PSD 정규화 (Min-Max Normalization)
- 2-2. adc1_buffer[0] 값에 이동평균 필터 적용
- 2-3. 원본값과 필터값 비교

Live Expression 화면은 따로 webm 동영상으로 GitHub에 업로드했다.

동영상 링크: https://naver.me/Fka0gOaN
(PSD와 Live Expression을 함께 촬영한 동영상)

---

## 3단계: 이동평균 필터를 4채널 전부(buffer[0] ~ [3])에 적용하기

### 핵심 아이디어

1. 반복되는 필터 로직을 **함수로 분리**한다.
2. 채널마다 **독립된 저장 공간**이 필요하다.
   - 채널 0의 최근 10개 값과 채널 1의 최근 10개 값이 섞이면 안 된다.
   - 채널 수만큼 별도의 버퍼가 있어야 한다.
   - → `filter_buffer[채널][최근값]` **2차원 배열**로 확장한다.

### 왜 채널별 저장 공간이 필요한가

이동평균은 "그 채널의 과거 값들"의 평균이다.
채널 0의 과거값과 채널 1의 과거값을 같은 배열에 섞으면
평균이 뒤죽박죽되어 필터의 의미가 사라진다.
그래서 각 채널이 자기만의 filter_buffer를 가져야 한다.

### 설계

**1) 변수 선언 (USER CODE BEGIN PV)**

```c
uint32_t filter_buffer[4][10] = {50,};   // [채널 4개][채널마다 최근 10개]
uint32_t filter_value[4] = {50,};        // 채널별 필터링 결과
```

**2) 필터 함수 (USER CODE BEGIN 0)**

역할: 새 값 하나를 받아서, 해당 채널 버퍼를 갱신하고 평균을 반환한다.

```c
uint32_t moving_average(uint32_t new_val, uint32_t buf[], int size)
{
    // 시프트 (뒤에서부터 앞 값을 당겨옴)
    for (int i = size - 1; i > 0; i--) {
        buf[i] = buf[i - 1];
    }
    buf[0] = new_val;   // 맨 앞에 새 값

    // 평균
    uint32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buf[i];
    }
    return sum / size;
}
```

**3) 메인 루프 (USER CODE BEGIN 3)**

```c
if (adc_updated_flag) {
    adc_updated_flag = 0;

    for (int ch = 0; ch < 4; ch++) {
        filter_value[ch] = moving_average(adc1_buffer[ch], filter_buffer[ch], 10);
    }
}
```

> `filter_buffer[ch]`는 2차원 배열의 한 행(그 채널의 10칸짜리 배열)을
> 함수에 넘기는 것이다. 함수 안에서는 `buf[]`로 받아 그 행만 다룬다.

### 동작 원리

```
adc1_buffer[0] ──▶ moving_average (buffer[0] 사용) ──▶ filter_value[0]
adc1_buffer[1] ──▶ moving_average (buffer[1] 사용) ──▶ filter_value[1]
adc1_buffer[2] ──▶ moving_average (buffer[2] 사용) ──▶ filter_value[2]
adc1_buffer[3] ──▶ moving_average (buffer[3] 사용) ──▶ filter_value[3]
```

함수 하나로 4채널을 모두 처리하고, 채널별 버퍼가 독립적으로 관리된다.

### 배운 점

- 중복되는 로직은 **함수로 뽑으면** 코드가 짧아지고 실수가 줄어든다.
- 채널별 상태는 **배열의 차원을 늘려** 분리한다.
- 이것이 다채널 센서 처리의 기본 패턴이다.

---

## 4단계: flag 방식 대신, buffer 인덱스 변화를 직접 감지하는 방법 설계

### 문제

지금은 DMA 완료 콜백이 adc_updated_flag를 세워주고,
메인 루프는 그 flag를 보고 처리한다.
flag(콜백)에 의존하지 않고, buffer가 갱신된 것을
직접 감지하는 다른 방법을 설계하라.



**값 자체의 변화 감지**
```c
uint32_t prev = 0;
if (adc1_buffer[0] != prev){
    prev = adc1_buffer[0];
}
```
직전 buffer 값을 저장해뒀다가, 현재 값과 다르면 갱신된 것으로 판단.
- 장점: 단순함
- 단점: 값이 우연히 이전과 똑같으면 갱신을 놓친다 (신뢰도 낮음)




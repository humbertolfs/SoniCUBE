#include <Arduino.h>
#include <ADCSampler.h>
#include <arduinoFFT.h>
#include <BleGamepad.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

BleGamepad bleGamepad("SoniCUBE", "Humberto Lopes {hlfs2@cin.ufpe.br}", 100);

void sendData(int note){
  switch (note)
  {
  //dpad left
  case 0:
    bleGamepad.setHat1(HAT_LEFT);
    delay(100);
    bleGamepad.setHat1(0);
    Serial.println("left pressed");
    break;

  //dpad down
  case 1:
    bleGamepad.setHat1(HAT_DOWN);
    delay(100);
    bleGamepad.setHat1(0);
    Serial.println("down pressed");
    break;

  //dpad right
  case 2:
    bleGamepad.setHat1(HAT_RIGHT);
    delay(100);
    bleGamepad.setHat1(0);
    Serial.println("right pressed");
    break;

  //dpad up
  case 3:
    bleGamepad.setHat1(HAT_UP);
    delay(100);
    bleGamepad.setHat1(0);
    Serial.println("up pressed");
    break;

  //button LT
  case 4:
    bleGamepad.press(BUTTON_9);
    delay(100);
    bleGamepad.release(BUTTON_9);
    Serial.println("LT pressed");
    break;

  //button LB
  case 5:
    bleGamepad.press(BUTTON_7);
    delay(100);
    bleGamepad.release(BUTTON_7);
    Serial.println("LB pressed");
    break;

  //button RB
  case 6:
    bleGamepad.press(BUTTON_8);
    delay(100);
    bleGamepad.release(BUTTON_8);
    Serial.println("RB pressed");
    break;

  //button RT
  case 7:
    bleGamepad.press(BUTTON_10);
    delay(100);
    bleGamepad.release(BUTTON_10);
    Serial.println("RT pressed");
    break;

  //button X
  case 8:
    bleGamepad.press(BUTTON_4);
    delay(100);
    bleGamepad.release(BUTTON_4);
    Serial.println("X pressed");
    break;

  //button A
  case 9:
    bleGamepad.press(BUTTON_1);
    delay(100);
    bleGamepad.release(BUTTON_1);
    Serial.println("A pressed");
    break;

  //button B
  case 10:
    bleGamepad.press(BUTTON_2);
    delay(100);
    bleGamepad.release(BUTTON_2);
    Serial.println("B pressed");
    break;

  //button Y
  case 11:
    bleGamepad.press(BUTTON_5);
    delay(100);
    bleGamepad.release(BUTTON_5);
    Serial.println("Y pressed");
    break;
  
  default:
    break;
  }
}

int frequencyToNoteName(float frequency) {
    // Nomes das notas musicais em uma oitava
    std::vector<std::string> noteNames = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
    };

    // Verificar se a frequência está dentro do intervalo válido
    if (frequency <= 0) {
        return -1;
    }

    // Cálculo do número de semitons em relação ao A4
    int semitonesFromA4 = std::round(12 * std::log2(frequency / 440.0));

    // Calcula a posição da nota na oitava
    int noteIndex = (semitonesFromA4-3) % 12;
    if (noteIndex < 0) {
        noteIndex += 12;
    }

    // Retorna o nome da nota musical com a oitava
    return noteIndex;
}

ADCSampler *adcSampler = NULL;
I2SSampler *i2sSampler = NULL;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 40000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};


// how many samples to read at once
const int SAMPLE_SIZE = 1024;

ArduinoFFT<float> FFT;

int count = 0;
int note = -1;

/**
 * This function is the task that reads samples from an I2S sampler and prints them to the serial monitor.
 *
 * @param param A pointer to the I2SSampler object.
 */
void adcWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  float *vReal = (float *)malloc(sizeof(float) * SAMPLE_SIZE);
  float *vImag = (float *)malloc(sizeof(float) * SAMPLE_SIZE);

  if (!samples)
  {
    Serial.println("Failed to allocate memory for samples");
    return;
  }

  while (true)
  {
    int samples_read = sampler->read(samples, vReal, vImag, SAMPLE_SIZE);

    FFT = ArduinoFFT<float>(vReal, vImag, samples_read, 40000.0);

    digitalWrite(2, HIGH);

    FFT.dcRemoval();
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    float peak = FFT.majorPeak();
    int newNote = frequencyToNoteName(peak);
    
    if(note != newNote){
      note = newNote;
      count = 0;
    } else if(newNote != -1){
      count++;
    } else {
      Serial.println("No note detected");
      count = 0;
    }
    
    if(bleGamepad.isConnected()){
      if(count >= 10){
        count = 0;
        Serial.println(note);
        sendData(note);
      }
    }
    
    digitalWrite(2, LOW);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleGamepad.begin();

  // indicator LED
  pinMode(2, OUTPUT);

  // create our sampler
  adcSampler = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_6, adcI2SConfig);

  // set up the adc sample writer task
  TaskHandle_t adcWriterTaskHandle;
  adcSampler->start();
  
  xTaskCreatePinnedToCore(adcWriterTask, "ADC Writer Task", 8192, adcSampler, 1, &adcWriterTaskHandle, 1);

  // start sampling from i2s device
}

void loop()
{
  // nothing to do here
}
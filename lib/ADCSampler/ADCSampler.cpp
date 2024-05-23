#include "ADCSampler.h"

ADCSampler::ADCSampler(adc_unit_t adcUnit, adc1_channel_t adcChannel, const i2s_config_t &i2s_config) : I2SSampler(I2S_NUM_0, i2s_config)
{
    m_adcUnit = adcUnit;
    m_adcChannel = adcChannel;
}

void ADCSampler::configureI2S()
{
    //init ADC pad
    i2s_set_adc_mode(m_adcUnit, m_adcChannel);
    // enable the adc
    i2s_adc_enable(m_i2sPort);
}

void ADCSampler::unConfigureI2S()
{
    // make sure not do this or the ADC is locked
    i2s_adc_disable(m_i2sPort);
}

int ADCSampler::read(int16_t *samples, float *vReal, float *vImag, int count)
{
    // read from i2s
    size_t bytes_read = 0;
    i2s_read(m_i2sPort, samples, sizeof(int16_t) * count, &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int16_t);
    for (int i = 0; i < samples_read; i++)
    {
        vReal[i] = static_cast<float>(uint16_t(samples[i]) & 0xfff);
        vImag[i] = 0.0;
    }
    return samples_read;
}
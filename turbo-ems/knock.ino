#define	TPIC8101_CS_PIN		7
#define	TPIC8101_HOLD_PIN	8
#define	TPIC8101_TEST_PIN	9
#define TPIC8101_OUT_PIN	A0
#define TPIC8101_INT_TIME 2000//time in microseconds to integrate knock

#define	SPU_SET_PRESCALAR_16MHz		0b01001100    /* 16MHz prescalar with SDO active */
#define	SPU_SET_CHANNEL_1			0b11100000    /* Setting active channel to 1 */
#define	SPU_SET_BAND_PASS_FREQUENCY	0b00101010    /* Setting band pass frequency to 7.27kHz */
#define	SPU_SET_PROGRAMMABLE_GAIN	0b10100010    /* Setting programmable gain to 0.381 */
#define	SPU_SET_INTEGRATOR_TIME		0b11001010 /* Setting programmable integrator time constant to 100µs */

#include <SPI.h>

inline static uint8_t spi_tx(uint8_t tx_data) {

    digitalWrite(TPIC8101_CS_PIN, LOW);
    byte Response = SPI.transfer(tx_data);
    digitalWrite(TPIC8101_CS_PIN, HIGH);

    return Response;
};

int8_t tpic8101_init()
{
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);

    pinMode(TPIC8101_CS_PIN, OUTPUT);
    pinMode(TPIC8101_HOLD_PIN, OUTPUT);
    pinMode(TPIC8101_TEST_PIN, OUTPUT);
    pinMode(TPIC8101_OUT_PIN, INPUT);

    digitalWrite(TPIC8101_CS_PIN, HIGH);
    digitalWrite(TPIC8101_TEST_PIN, HIGH);
    digitalWrite(TPIC8101_HOLD_PIN, LOW);

    if(spi_tx(SPU_SET_PRESCALAR_16MHz)!=SPU_SET_PRESCALAR_16MHz)return false;
    else if(spi_tx(SPU_SET_CHANNEL_1)!=SPU_SET_CHANNEL_1)return false;
    else if(spi_tx(SPU_SET_BAND_PASS_FREQUENCY)!=SPU_SET_BAND_PASS_FREQUENCY)return false;
    else if(spi_tx(SPU_SET_PROGRAMMABLE_GAIN)!=SPU_SET_PROGRAMMABLE_GAIN)return false;
    else if(spi_tx(SPU_SET_INTEGRATOR_TIME)!=SPU_SET_INTEGRATOR_TIME)return false;
    return true;
};

uint16_t tpic8101_get(uint32_t curr_time)
{
    static uint32_t last_time=0;
    static uint16_t last_read=0;
    if(last_time == 0)
    {
        last_time=curr_time;
        digitalWrite(TPIC8101_HOLD_PIN, HIGH);
    }
    else if( (curr_time-last_time) >= TPIC8101_INT_TIME )
    {
        last_time=0;
        last_read=analogRead(TPIC8101_OUT_PIN);
        digitalWrite(TPIC8101_HOLD_PIN, LOW);
    }     
    return last_read;
};

/**
 * @file    main.c
 * @brief   LED Effects Demo for EFM32GG_STK3700
 * @version 1.0
 *
 * @note    Blink 8 LEDs with different effects 
 *
 * @note    LEDs are on pins 0 to 8 of GPIO Port D
 *
 * @note    It uses a primitive delay mechanism.
 *
 * @author  Henrique Faria Ribeiro / Henrique Paulino Cruz
 * @date    25/01/2022
 */

#include <stdint.h>
#include "em_device.h"

/**
 * @brief       BIT macro
 *              Defines a constant (hopefully) with a bit 1 in the N position
 * @param       N : Bit index with bit 0 the Least Significant Bit
 */
#define BIT(N) (1U << (N))

#define BUTTON1 BIT(9)
#define BUTTON2 BIT(10)

#define SWITCH BIT(0)

/// Default delay value.
#define DELAYVAL 3
/**
 * @brief  Quick and dirty delay function
 * @note   Do not use it in production code
 */

void button_init(uint32_t buttons);
void switch_init(uint32_t buttons);
uint32_t button_read_pressed(void);
int update_buttons();
void zera_leds(GPIO_P_TypeDef *const GPIOD);
void delay(uint32_t delay);
void incremento(GPIO_P_TypeDef *const GPIOD);
void decremento(GPIO_P_TypeDef *const GPIOD);
void desloca_direita(GPIO_P_TypeDef *const GPIOD);
void desloca_esquerda(GPIO_P_TypeDef *const GPIOD);
void equalizador(GPIO_P_TypeDef *const GPIOD);
void liga_desliga(GPIO_P_TypeDef *const GPIOD);
void auto_fantastico(GPIO_P_TypeDef *const GPIOD);
void propria(GPIO_P_TypeDef *const GPIOD);


static GPIO_P_TypeDef * const GPIOB = &(GPIO->P[1]);    // GPIOB
static GPIO_P_TypeDef * const GPIOC = &(GPIO->P[2]);    // GPIOC
static GPIO_P_TypeDef * const GPIOD = &(GPIO->P[3]);    // GPIOD


static uint32_t lastread = 0;
static uint32_t inputpins = 0;

static char buttons[3] = {0,0,0};


/**
 * @brief  Main function
 *
 * @note   Using default clock configuration
 * @note   HFCLK     = HFRCO 14 MHz
 * @note   HFCORECLK = HFCLK
 * @note   HFPERCLK  = HFCLK

 */
 
 /*
    SWITCH BUTTON2 BUTTON1

    000 Incremento
    001 Decremento
    010 Deslocamento à direita de 1 led
    011 Deslocamento à esquerda de 1 led
    100 Equalizador
    101 Liga/desliga
    110 Auto fantástico
    111 Própria
 */

int main(void){
    /* Enable Clock for GPIO */
    CMU->HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN; // Enable HFPERCLK
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;       // Enable HFPERCKL for GPIO

    /* Configure Pins in GPIOD */
    GPIOD->MODEL = (0x44444444); // Set bits

    /* Initial values for LEDs */
    GPIOD->DOUT = 0U; // Turn Off LEDs


    /* Initialize Buttons */
    button_init(BUTTON1|BUTTON2);

    /* Initialize Switch */
    switch_init(SWITCH);

    /* loop */
    while (1) {
        update_buttons();

        if(!buttons[0] && !buttons[1] && !buttons[2])
            incremento(GPIOD);
        else if(buttons[0] && !buttons[1] && !buttons[2])
            decremento(GPIOD);
        else if(!buttons[0] && buttons[1] && !buttons[2])
            desloca_direita(GPIOD);
        else if(buttons[0] && buttons[1] && !buttons[2])
            desloca_esquerda(GPIOD);
        else if(!buttons[0] && !buttons[1] && buttons[2])
            equalizador(GPIOD);
        else if(buttons[0] && !buttons[1] && buttons[2])
            liga_desliga(GPIOD);
        else if(!buttons[0] && buttons[1] && buttons[2])
            auto_fantastico(GPIOD);
        else if(buttons[0] && buttons[1] && buttons[2])
            propria(GPIOD);
    }
}

void delay(uint32_t delay) {
    volatile uint32_t counter;
    int i;

    for (i = 0; i < delay; i++) {
        counter = 100000;
        while (counter)
            counter--;
    }
}

void zera_leds(GPIO_P_TypeDef *const GPIOD) {
    GPIOD->DOUT &= 0U;
}

void incremento(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);

    for(unsigned int i = 0; i < 255; i++) {
        GPIOD->DOUT = i;
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }

    zera_leds(GPIOD);
}

void decremento(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);

    for(unsigned int i = 255; i >= 0; i--) {
        GPIOD->DOUT = i;
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }

    zera_leds(GPIOD);
}

void desloca_direita(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);
    
    for(int i = 0; i < 8; i++) {
        GPIOD->DOUT = BIT(i);
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }
    zera_leds(GPIOD);
}

void desloca_esquerda(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);
    
    for(int i = 7; i >= 0; i--) {
        GPIOD->DOUT = BIT(i);
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }
    zera_leds(GPIOD);
}

void equalizador(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);
    
    for(int i = 0; i < 8; i++) {
        GPIOD->DOUT ^= BIT(i);
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }

    for(int i = 7; i >= 0; i--) {
        GPIOD->DOUT ^= BIT(i);
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }

    zera_leds(GPIOD);
}

void liga_desliga(GPIO_P_TypeDef *const GPIOD) {
    unsigned int out = 0;

    zera_leds(GPIOD);
    delay(DELAYVAL);
    
    for(int i = 0; i < 8; i++) {
        out |= BIT(i);
        if(update_buttons())
            break;
    }
    GPIOD->DOUT = out;
    delay(DELAYVAL);
}

void auto_fantastico(GPIO_P_TypeDef *const GPIOD) {
    int qtd = 3;
    unsigned int group = 0;

    for(int i = 0; i < qtd; i++){
        group |= BIT(i);
    }

    int dif = 8 - qtd;

    GPIOD->DOUT = group;

    for(int i = 0; i < dif; i++){
        group = group << 1;
        GPIOD->DOUT = group;
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }

    for(int i = dif; i > 0; i--){
        group = group >> 1;
        GPIOD->DOUT = group;
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }
}

void propria(GPIO_P_TypeDef *const GPIOD) {
    zera_leds(GPIOD);
    for(int i = 0; i < 4; i++){
        GPIOD->DOUT ^= (BIT(i) | BIT(7-i));
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }
    for(int i = 0; i < 4; i++){
        GPIOD->DOUT ^= (BIT(i) | BIT(7-i));
        delay(DELAYVAL);
        if(update_buttons())
            break;
    }
    zera_leds(GPIOD);
}

void button_init(uint32_t buttons) {

    if ( buttons&BUTTON1 ) {
        GPIOB->MODEH &= ~_GPIO_P_MODEL_MODE1_MASK;      // Clear bits
        GPIOB->MODEH |= GPIO_P_MODEL_MODE1_INPUT;       // Set bits
        inputpins |= BUTTON1;
    }

    if ( buttons&BUTTON2 ) {
        GPIOB->MODEH &= ~_GPIO_P_MODEL_MODE2_MASK;      // Clear bits
        GPIOB->MODEH |= GPIO_P_MODEL_MODE2_INPUT;       // Set bits
        inputpins |= BUTTON2;
    }

    // First read
    lastread = GPIOB->DIN;

}

void switch_init(uint32_t _switch) {
    if ( _switch&SWITCH ) {
        GPIOC->MODEL &= ~_GPIO_P_MODEL_MODE0_MASK;              // Clear bits
        GPIOC->MODEL |= GPIO_P_MODEL_MODE0_INPUTPULLFILTER;     // Set bits
    }
}

uint32_t button_read_pressed(void) {
    uint32_t newread;
    uint32_t changes;

    newread = GPIOB->DIN;
    changes = ~newread&lastread;
    lastread = newread;

    return changes&inputpins;
}

int update_buttons() {
    uint32_t b = button_read_pressed();
    int changed = 0;
    
    if(b&BUTTON1){
        buttons[0] = !buttons[0];
        changed = 1;
    }

    if(b&BUTTON2){
        buttons[1] = !buttons[1];
        changed = 1;
    }

    int antes = buttons[2];
    buttons[2] = GPIOC->DIN&SWITCH;
    if(buttons[2] != antes){
        changed = 1;
    }

    return changed;
}
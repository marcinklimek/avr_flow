/*

 28 - we W1
 27 - led blink
 26 - we W2
 25 - impulsy z wrzutnika
 24 - stop
 23 - we W3
 
 15 - wy pompa
 
 HC595
 4 PD2 - we    
 5 PD3 - zapis enable 
 6 PD4 - clk


10 - btn MEM
12 - "-"
13 - "+"

--2--
3   4
--6--
1   5
--7--

P 
   1234 67 
0b10000101 

0-9
   1234567
0b10000010,
0b11110011,
0b11010000,
0b10010100,
0b11100001,
0b11001000,
0b10001000,
0b11010011,
0b10000000,
0b11000001
*/

#include <PinChangeInt.h>
#include "TimerOne/TimerOne.h"
#include <Button.h>
#include <EEPROMex.h>

#define PULLUP true
#define INVERT true

typedef void (*voidFuncPtr)(void);

#define FLOW_SENSOR_PIN 8

#define COIN_1_PIN 9
#define COIN_2_PIN 8
#define COIN_3_PIN 7

#define COIN_1 0
#define COIN_2 1
#define COIN_3 2

#define STOP_PIN   8
#define PUMP_PIN   9 //3

#define LED_PIN    10 //2
#define LED_BLINK_OFF          0
#define LED_BLINK_SUPER_FAST 100
#define LED_BLINK_FAST       200
#define LED_BLINK_NORMAL     500

/* LED segment */
#define DATA_PIN 2     //PD2
#define LATCH_PIN 3    //PD3
#define CLOCK_PIN 4    //PD4

const byte dec_digits[] = 
    { 
    0b10000010,
    0b11110011,
    0b11010000,
    0b10010100,
    0b11100001,
    0b11001000,
    0b10001000,
    0b11010011,
    0b10000000,
    0b11000001 
    };
    
const byte symbols[]    = {0b10000101, 0b1001000};  // 0 - P
const byte segments[]   = { 0b01111111, 
                            0b10111111, 
                            0b11011111, 
                            0b11101111, 
                            0b11110111 };
byte menu_idx = 0;


uint8_t segment_data[4];
uint8_t segment = 0;

uint16_t number_tokens_for_one = 146;

uint16_t cumulative_number_coins = 0;

uint8_t pump_state = 0;  
volatile int16_t flow_pulses = 0;

volatile int coins_counter[COIN_3+1]={0,0,0};
unsigned long previous_millis_coin = 0;

/* Status blinker */
uint8_t led_state; 
unsigned long led_blink_interval = LED_BLINK_OFF;

unsigned long previous_millis_led = 0;
unsigned long previous_millis_console= 0;
unsigned long previous_millis_tokens= 0;
unsigned long previous_millis_inactivity = 0;
unsigned long previous_millis_flow = 0;

  
// 2 min
#define INACTIVITY_TIMEOUT 120


/* MENU */
#define LONG_PRESS 1000
#define MENU_DOWN_PIN 9  
#define MENU_ENTER_PIN 8 
#define MENU_UP_PIN 7  

enum {MENU_WAIT = 0, MENU_SET_TOKENS, MENU_STAT, MENU_SAVE};  
    
uint8_t menu_state = MENU_WAIT;

Button btn_enter(MENU_ENTER_PIN, PULLUP, INVERT, 20);
Button btn_up(MENU_UP_PIN, PULLUP, INVERT, 20);
Button btn_down(MENU_DOWN_PIN, PULLUP, INVERT, 20);

#define REPEAT_FIRST 500   //ms required before repeating on long press
#define REPEAT_INCR  100   //repeat interval for long press
#define MIN_COUNT      1
#define MAX_COUNT    999

enum {BUTTON_UP_DOWN_WAIT, BUTTON_UP_DOWN_INCR, BUTTON_UP_DOWN_DECR};
    
uint8_t up_dn_state;                  //The current state machine state
volatile uint16_t temporary_token_price = 0;            
int lastCount = -1;                   //Previous value of count (initialized to ensure it's different when the sketch starts)
unsigned long rpt = REPEAT_FIRST;     //A variable time that is used to drive the repeats for long presses


void set_segment_data(uint8_t* digit, int16_t val)
{
    if (val < 0)
        val = 0;
        
    digit[0] = val/1000;
    digit[1] = (val%1000)/100;
    digit[2] = (val%100)/10;
    digit[3] = (val%100)%10;
}
   
   
void draw_led()
{
    uint8_t value;
    
    if ( segment == 4 ) // menu leds
        value = segments[menu_idx];
    else
        value = dec_digits[ segment_data[segment] ];

    bitClear(PORTD, LATCH_PIN); //latch off
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, value);
    bitSet(PORTD, LATCH_PIN); //latch on
    
    bitSet(PORTB, 6);
    bitSet(PORTB, 7);
    bitSet(PORTD, 5);
    bitSet(PORTD, 6);
    bitSet(PORTD, 7);

    if      (segment == 0)
        bitClear(PORTB, 7);
    else if (segment == 1) 
        bitClear(PORTD, 5);
    else if (segment == 2)
        bitClear(PORTD, 6);
    else if (segment == 3)
        bitClear(PORTD, 7);
    else if (segment == 4)
        bitClear(PORTB, 6);
        
    segment++;
    if ( segment == 5 )
        segment = 0;
}

void start()
{
    flow_pulses = 0;
    pump_state = 1;
        
    led_blink_interval = LED_BLINK_NORMAL;
        
    digitalWrite(PUMP_PIN, HIGH);
        
    Serial.print( "START\n\r" ); 
        
    previous_millis_led = 0;
    previous_millis_console= 0;
    previous_millis_tokens= 0;
    previous_millis_inactivity = millis();
}

void stop()
{
    digitalWrite(PUMP_PIN, LOW);
    
    led_blink_interval = LED_BLINK_OFF;
    led_state = HIGH;
    digitalWrite(LED_PIN, led_state);

    noInterrupts();
    
    pump_state = 0;
    flow_pulses = 0;
        
    coins_counter[COIN_1] = 0;
    coins_counter[COIN_2] = 0;
    coins_counter[COIN_3] = 0;
    
    
    previous_millis_led = 0;
    previous_millis_console= 0;
    previous_millis_tokens= 0;
   
    previous_millis_inactivity = 0;
    
    interrupts();
    
    //zapisanie ilosci na koniec dzialania
    EEPROM.writeInt(2, cumulative_number_coins);
    
    Serial.print( "STOP\n\r" ); 
}

void callback_flowPin()
{
    unsigned long curr_time = millis();
    
    if ( abs(curr_time - previous_millis_flow) > 15 )
    {
        flow_pulses++;
    
        previous_millis_flow = curr_time;
    }
    
    previous_millis_inactivity = curr_time;
}

void callback_coinPin()
{
    switch( PCintPort::arduinoPin )
    {
        case COIN_1_PIN:
        {
            coins_counter[COIN_1]++;
            cumulative_number_coins += 1;
            break;
        }        
        case COIN_2_PIN:
        {
            coins_counter[COIN_2]++;
            cumulative_number_coins += 2;
            break;
        }        
        case COIN_3_PIN:
        {
            coins_counter[COIN_3]++;
            cumulative_number_coins += 5;
            break;
        }                                   
    }            
}

void callback_stopPin()
{
    stop();
}

int16_t calculate_tokens()
{
    return coins_counter[COIN_1] * number_tokens_for_one +     // 1pln
           coins_counter[COIN_2] * number_tokens_for_one*2 +   // 2pln
           coins_counter[COIN_3] * number_tokens_for_one*5;    // 5pln 
}

void setup_callback(uint8_t pin, voidFuncPtr callback_ptr)
{
    PCintPort::attachInterrupt(pin, callback_ptr, FALLING);
    pinMode(pin, INPUT);
}

void setup() 
{
    Serial.begin(9600);
    Serial.println("Dystrybutor, setup start");
  
    DDRB = DDRB | (1<<DDB7);// | (1<<DDB6);
    DDRD = (1<<DDB5) |  (1<<DDB6) | (1<<DDB7);
    
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
  
    digitalWrite(LATCH_PIN, HIGH);
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(DATA_PIN, HIGH);
  
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    setup_callback(FLOW_SENSOR_PIN, &callback_flowPin);
    
    setup_callback(COIN_1_PIN, &callback_coinPin);
    setup_callback(COIN_2_PIN, &callback_coinPin);
    setup_callback(COIN_3_PIN, &callback_coinPin);
    setup_callback(STOP_PIN, &callback_stopPin);

    //Timer1.initialize(5000);
    //Timer1.attachInterrupt( draw_led );
    
    while (!EEPROM.isReady()) { delay(1); }
    number_tokens_for_one = EEPROM.readInt(0);
    
    temporary_token_price = number_tokens_for_one;
    
    cumulative_number_coins = EEPROM.readInt(2);
    
    menu_state = MENU_WAIT;

    up_dn_state = BUTTON_UP_DOWN_WAIT;
    
    Serial.println("Dystrybutor, setup end");
    
    stop();
}

void process_menu(int16_t current_tokens)
{
    btn_enter.read();
    btn_up.read();
    btn_down.read();
        
	switch( menu_state )
    {
        case MENU_WAIT:
        {
            if ( btn_enter.pressedFor( LONG_PRESS ) )
            {
                Serial.println( "[MENU] Entered" ); 
                menu_state = MENU_SET_TOKENS;
                temporary_token_price = number_tokens_for_one;
            }                
            
            if (btn_up.wasPressed())
            {
                int pump_state = digitalRead(PUMP_PIN);
                if (pump_state == HIGH)
                    pump_state = LOW;
                else
                    pump_state = HIGH;
                               
                digitalWrite(PUMP_PIN, pump_state);
            }
            
            break;
       }       
       case MENU_SET_TOKENS:
       {
            if ( btn_enter.wasPressed() )
            {
                menu_state = MENU_STAT;
                break;
            }                
                
            switch (up_dn_state) 
            {
                case BUTTON_UP_DOWN_WAIT:                   //wait for a button event
                    if (btn_up.wasPressed())
                        up_dn_state = BUTTON_UP_DOWN_INCR;
                    else if (btn_down.wasPressed())
                        up_dn_state = BUTTON_UP_DOWN_DECR;
                    else if (btn_up.wasReleased())          //reset the long press interval
                        rpt = REPEAT_FIRST;
                    else if (btn_down.wasReleased())
                        rpt = REPEAT_FIRST;
                    else if (btn_up.pressedFor(rpt))        //check for long press
                    {     
                        rpt += REPEAT_INCR;                 //increment the long press interval
                        up_dn_state = BUTTON_UP_DOWN_INCR;
                    }
                    else if (btn_down.pressedFor(rpt)) 
                    {
                        rpt += REPEAT_INCR ;
                        up_dn_state = BUTTON_UP_DOWN_DECR;
                    }
                    break;

                case BUTTON_UP_DOWN_INCR:        
                {
                    int t = temporary_token_price + 1;                                                       ;
                    temporary_token_price = min(t, MAX_COUNT);      //but not more than the specified maximum
                    up_dn_state = BUTTON_UP_DOWN_WAIT;
                    
                    Serial.print( "[MENU] inc " ); 
                    Serial.println( temporary_token_price ); 

                    break;
                }
                
                case BUTTON_UP_DOWN_DECR:                               
                {
                    int t = temporary_token_price - 1;
                    temporary_token_price = max(t, MIN_COUNT);      //but not less than the specified minimum
                    up_dn_state = BUTTON_UP_DOWN_WAIT;
                    
                    Serial.print( "[MENU] dec "  );
                    Serial.println( temporary_token_price );
                    
                    break;
                }                    
            }
                
            break;
       }            
       case MENU_STAT:
       {
            if (btn_down.pressedFor(500)) 
            {
                cumulative_number_coins = 0;
            }
            if ( btn_enter.wasPressed() )
            {
                menu_state = MENU_SAVE;
                break;
            }
       
            break;
       }
       case MENU_SAVE:
       {
            menu_state = MENU_WAIT;
            
            if ( number_tokens_for_one != temporary_token_price )
            {   
                number_tokens_for_one = temporary_token_price;
    
                while (!EEPROM.isReady()) { delay(1); }
                EEPROM.writeInt(0, number_tokens_for_one);
                EEPROM.writeInt(2, cumulative_number_coins);
                
                Serial.println("[MENU] saved");
            }   
             
            break;
       }            
    }
    
    if ( menu_state == MENU_WAIT )
    {
        set_segment_data(segment_data, current_tokens - flow_pulses);
    } 
    else if ( menu_state == MENU_SET_TOKENS)
    {
       set_segment_data(segment_data, temporary_token_price); 
    }
    else if ( menu_state == MENU_STAT )
    {
        set_segment_data(segment_data, cumulative_number_coins);
    }


}

void blinker(unsigned long current_millis)
{
	if( (led_blink_interval > 0) && (current_millis - previous_millis_led) >= led_blink_interval) 
    {
        previous_millis_led = current_millis;
        
        if (led_state == LOW)
            led_state = HIGH;
        else
            led_state = LOW;
            
        digitalWrite(LED_PIN, led_state);
    }
}

boolean terminal_output(unsigned long current_millis)
{
	if( (current_millis - previous_millis_console) >= 1000UL)
    {
        previous_millis_console = current_millis;
        
        Serial.print( millis() ); Serial.print( " - " );
        
        Serial.print( "C1 " ); Serial.print( coins_counter[COIN_1] ); Serial.print( " " );
        Serial.print( "C2 " ); Serial.print( coins_counter[COIN_2] ); Serial.print( " " );
        Serial.print( "C5 " ); Serial.print( coins_counter[COIN_3] ); Serial.print( " " );
        
        Serial.print( "Flow " ); Serial.print( flow_pulses ); Serial.print( " " );

        int16_t current_tokens = calculate_tokens();
        int16_t tokens_diff = current_tokens - flow_pulses;
        
        Serial.print( "diff " );  Serial.print( tokens_diff ); 

        Serial.println();
       
        return true;
    }	
    
    return false;
}

void inactivity_check(unsigned long current_millis)
{
	unsigned long inactivity_diff = 0;
    if ( previous_millis_inactivity < current_millis )
    {
        inactivity_diff = (current_millis - previous_millis_inactivity);
        inactivity_diff = inactivity_diff/1000UL;
    }
    
    if ( inactivity_diff > INACTIVITY_TIMEOUT ) 
    {
        Serial.println( "Timeout" );
        stop();
    }
}

void loop()
{
    unsigned long current_millis = millis();

    draw_led();

    volatile int current_tokens = calculate_tokens();
    
    if ( current_tokens > 0 )
    {
        int tokens_diff = current_tokens - flow_pulses;

        if ( tokens_diff > 0 )
        {
            if ( !pump_state ) 
                start();
        }            
        else
            stop();
            
        if ( (float)flow_pulses/(float)current_tokens > 0.85f )
            led_blink_interval = LED_BLINK_SUPER_FAST;
        else if ( (float)flow_pulses/(float)current_tokens > 0.75f )
            led_blink_interval = LED_BLINK_FAST; 
    
        inactivity_check(current_millis);               
    }
    
    // -- DEBUG --------------------------------------------------------------------
    boolean output_to_console = terminal_output(current_millis);
           
    if ( output_to_console )
    {
        Serial.print( "tokens " ); Serial.print( current_tokens ); Serial.print( " " );
        Serial.print( previous_millis_inactivity ); Serial.print(" ");
        
        Serial.print( "\n\rMenu state: " );
        Serial.println(menu_state);
        
        //heart_beat();

        menu_idx++;
        if (menu_idx==5)
            menu_idx = 0;
            
        Serial.println( menu_idx );
    }    
    
    process_menu(current_tokens);
    blinker(current_millis);    
}
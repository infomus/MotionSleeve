#define UBRRVAL           103   //9600 baud rate to comply with HC_05

//cursor positions
#define MANGLE_POS        007
#define UYAW_POS          115
#define UPITCH_POS        015
#define UROLL_POS         007

//rx flags
#define UDATA_FLAG        240
#define MDATA_FLAG        (0xFF^UDATA_FLAG)

//current flags
#define MAIN_CURR         0
#define YAW_CURR          1
#define PITCH_CURR        2
#define ROLL_CURR         3

#define INACTIVE_THRESH   255   //match threshold on motion sleeve

typedef enum {
  REALTIME;
  HISTORY;
  //GOALS;
} program_state;

program_state CURR_STATE;

uint8_t cursor_pos;
uint8_t curr_angle;

uint8_t mangle_max;
uint8_t uyaw_max;
uint8_t upitch_max;
uint8_t uroll_max;

void UART_init(void);
void angle_rx(void);
void lcd_init(void);

void check_lcd_busy(void);
void flash_lcd(void);
void send_cmd(unsigned char command);
void disp_char(unsigned char character);
uint8_t get_pos(uint8_t);

int main(void)
{
  uint8_t char_data;
  uint8_t target_pos;
  UART_Init();
  lcd_init();
  while(1)
  {
    while(!(UCSRA&(1<<RXC)))
    {
    }
    curr_data=UDR;

    if (CURR_STATE == REALTIME)
    {
      target_pos = get_pos(curr_data);
  
      while (cursor_pos != target_pos)
      {
        if (cursor_pos> target_pos) 
        {
          send_cmd(0x10);
          cursor_pos--;
        }
        if (cursor_pos < target_pos) 
        {
          send_cmd(0x14);
          cursor_pos++;
        }
      }

      print_3dig(curr_data);

      //get max
      switch(cursor_pos)
      {
        case MANGLE_POS:
          if (curr_data > mangle_max) mangle_max = curr_data;
         case UYAW_POS:
          if (curr_data > uyaw_max) uyaw_max = curr_data;
        case UPITCH_POS:
          if (curr_data > upitch_max) upitch_max = curr_data;
        case UROLL_POS:
          if (curr_data > uroll_max) uroll_max = curr_data;
      }
    } else if (CURR_STATE == HISTORY)
    { 
      target_pos = get_pos(curr_data);
      switch(target_pos)
      {
        case MANGLE_POS:
          if (curr_data > mangle_max) 
          {
            mangle_max = curr_data;
            cursor_pos = MANGLE_POS;
            print_3dig(curr_data);
          }
         case UYAW_POS:
          if (curr_data > uyaw_max) 
          {
            uyaw_max = curr_data;
            cursor_pos = UYAW_POS;
            print_3dig(curr_data);
          }
        case UPITCH_POS:
          if (curr_data > upitch_max) 
          {
            upitch_max = curr_data;
            cursor_pos = UPITCH_POS;
            print_3dig(curr_data);
          }
        case UROLL_POS:
          if (curr_data > uroll_max) 
          {
            uroll_max = curr_data;
            cursor_pos = UROLL_POS;
            print_3dig(curr_data);
          }
      }
    }
  }

  return 0;
}

void check_lcd_busy()
{
  DDRB = 0;
  PORTD |= 1<<7;
  PORTD &= ~1<<2;
  
  while (PORTB >= 0x80)
  {
    flash_lcd();
  }
  DDRB = 0xFF;
}

void flash_lcd()
{
  // changes take affect after display is reset
  PORTD |= 1<<5;
  // wait 500ns
  asm volatile ("nop");
  asm volatile ("nop");
  PORTD &= ~1<<5;
}

void send_cmd(unsigned char command)
{
  check_lcd_busy();
  PORTB = command;
  PORTD &= ~ ((1<<7)|(1<<2));
  flash_lcd();
  PORTB = 0;
}

void disp_char(unsigned char character)
{
  check_lcd_busy();
  PORTB = character;
  PORTD &= ~ (1<<7);
  PORTD |= 1<<2;
  flash_lcd();
  PORTB = 0;
}

void lcd_init()
{
  const char *p = "MAIN:0 PITCH:0 ";
  DDRD |= 1<<5 | 1<<7 | 1<<2;
  //clear screen
  send_cmd(0x01);
  send_cmd(0x38);
  send_cmd(0x0E);
  
  while (*p != null)
  {
    disp_char(*p);
    p++;
  }
  while (cursor_pos != 0x40)
  {
    send_cmd(0x14);
    cursor_pos++;
  }

  *p = "ROLL:0 YAW:0 ";

   while (*p != null)
  {
    disp_char(*p);
    p++;
  }
}

uint8_t get_pos(uint8_t curr_data)
{
  curr_angle = MAIN_CURR;

  if (curr_data & 0xF0) curr_angle = YAW_CURR;
  else if (curr_data & 0x0F) curr_angle = MAIN_CURR;
  else 
  {
    if (curr_angle > MAIN_CURR && curr_angle <= ROLL_CURR) curr_angle++;
    else if (curr_angle > MAIN_CURR) err_handler();
  }
  
  switch(curr_angle)
  {
    case MAIN_CURR:
      return MANGLE_POS;
     case YAW_CURR:
      return UYAW_POS;
    case PITCH_CURR:
      return UPITCH_POS;
    case ROLL_CURR:
      return UROLL_POS;
    default:
      return MAIN_CURR;
  }
}

void print_3dig (uint8_t target_data)
{
  //display each character
  disp_char((target_data)%10);
  send_cmd(0x10);
  disp_char((target_data/10)%10);
  send_cmd(0x10);
  disp_char((target_data/100)%10);
  send_cmd(0x10);
  
  //reset to last digit so current position can be mapped
  send_cmd(0x14);
  send_cmd(0x14);
  send_cmd(0x14);
}

void UART_Init()
{
  //Set baud rate
  UBRRL=UBRRVAL;
  UBRRH=(UBRRVAL>>8);
  UCSRC=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
  UCSRB=(1<<RXEN);
}

ISR(INT0_vect)
{
  if (CURR_STATE == REALTIME) CURR_STATE = HISTORY;
  if (CURR_STATE == HISTORY) CURR_STATE = REALTIME;
  //if (CURR_STATE == GOALS) CURR_STATE = REALTIME;
}

void err_handler()
{
  //TODO display err msg, prompt reset
}

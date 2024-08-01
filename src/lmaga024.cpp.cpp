#include "timerISR-Fixed.h"
#include "helper.h"
#include "periph.h"
#include "serialATmega.h"
#include "MAX7219.h"
#include "LCD.h"

#define NUM_TASKS 4 // TODO: Change to the number of tasks being used

typedef struct _task
{
  signed char state;         // Task's current state
  unsigned long period;      // Task period
  unsigned long elapsedTime; // Time elapsed since last task tick
  int (*TickFct)(int);       // Task tick function
} task;

task tasks[NUM_TASKS] = {}; // declared task array with 5 tasks

unsigned char gameGrid[8] =
    {0b11100011, // Turn on LEDs except middle 3 columns
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11101011,
     0b11100011};

unsigned char MovingGameGrid[8] =
    {0b11100011, // Turn on LEDs except middle 3 columns
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011};

unsigned char ResetGrid[8] =
    {0b11100011, // Turn on LEDs except middle 3 columns
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011,
     0b11100011};

unsigned char carGrid[8] =
    {0b00000000, // Turn on LEDs except middle 3 columns
     0b00000000,
     0b00000000,
     0b00000000,
     0b00000000,
     0b00000000,
     0b00001000,
     0b00000000}; // Represents the car's position
unsigned char ResetCarGrid[8] =
    {0b00000000, // Turn on LEDs except middle 3 columns
     0b00000000,
     0b00000000,
     0b00000000,
     0b00000000,
     0b00000000,
     0b00001000,
     0b00000000};

unsigned char EndGameGrid[8] =
    {0b11111111, // Turn on LEDs except middle 3 columns
     0b11111111,
     0b11111111,
     0b11111111,
     0b11111111,
     0b11111111,
     0b11111111,
     0b11111111};
// ===========================================================================================================================================
const unsigned long GCD_PERIOD = 1;
const unsigned long Buzzer_Period = 1;
unsigned long Matrix_Period = 200;
const unsigned long Joyststick_Period = 100;
const unsigned long LCD_Period = 1000;

// ===========================================================================================================================================
int Buzzer_Tick(int state);
int MatrixDisplay_Tick(int state);
int Joystick_Tick(int state);
int LCD_Tick(int state);

//  ===========================================================================================================================================

enum Buzzer_states
{
  sm_BuzzIdle,
  sm_PlayNote,
  sm_NextNote,
  sm_EndGameMusic
} Buzzer_state;

enum MatrixDisplay_Tick
{
  sm_StartDisplay,
  sm_GameDisplay,
  sm_ResetDisplay
} MDisplay_state;

enum Joystick_Tick
{
  sm_Idle,
  sm_Left,
  sm_Right
} Joystick_state;

enum LCD_states
{
  sm_Display
} LCD_state;

//  ===========================================================================================================================================

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

void updateWallPosition()
{
  for (unsigned char i = 0; i < 8; i++)
  {

    MovingGameGrid[i] = gameGrid[i];
  }
}
//

//
void updateCarPosition()
{
  for (unsigned char i = 0; i < 8; i++)
  {
    Matrix_write(MAX7219_REG_DIGIT0 + i, MovingGameGrid[i] | carGrid[i]);
  }
}
//
int FirstRunL = 0;
int FirstRunR = 0;
int cnt = 0;
int cnt1 = 0;
int currScore = -1;
int HighScore = 0;
int Start = 0;
int Restart = 0;
int currCnt = 0;
unsigned char carPos = 4; // Initial position of the car in column 4
int p = 0;

void Shifting(char Direction)
{

  if (Direction == 'L' && FirstRunL < 26)
  {
    // serial_println(FirstRunL);
    if (p < 8)
    {
      MovingGameGrid[p] = (MovingGameGrid[p] << 1) | 0b00000001; // Shift Left once and set far left bit
      Matrix_write(MAX7219_REG_DIGIT0 + p, MovingGameGrid[p]);
      p++;
    }
    else if (p >= 8)
    {
      p = 0;
    }
    FirstRunL++;
  }
  else if (Direction == 'R' && FirstRunR < 45)
  {

    // serial_println(FirstRunR);
    if (p < 8)
    {
      MovingGameGrid[p] = (MovingGameGrid[p] >> 1) | 0b10000000; // Shift right once and set far left bit
      Matrix_write(MAX7219_REG_DIGIT0 + p, MovingGameGrid[p]);
      p++;
    }
    else if (p >= 8)
    {
      p = 0;
    }
    FirstRunR++;
  }

  if (FirstRunL > 26 && FirstRunR > 45)
  {
    FirstRunL = 0;
    FirstRunR = 0;
  }
}
int GameState = 0;
int EndGame = 0;
unsigned int noteIndex = 0; // Current note index in the melody
unsigned Buzzercnt = 0;
unsigned int deadIndex = 0;

int MatrixDisplay_Tick(int MDisplay_state)
{
  static int counter1 = 0;
  static int counter = 0;
  static int RealCounterL = 0;
  static int RealCounterR = 0;
  static int j = 0;
  ;
  switch (MDisplay_state)
  {
  case sm_StartDisplay:
    if (GameState == 1)
    {
      MDisplay_state = sm_GameDisplay;
    }
    break;
  case sm_GameDisplay:
    if (Restart >= 1)
    {
      MDisplay_state = sm_ResetDisplay;
    }
    break;
  case sm_ResetDisplay:
    if (GameState == 0)
    {
      MDisplay_state = sm_StartDisplay;
    }
    break;
  }

  // ACTIONSSSSSSSSSSSSSS
  switch (MDisplay_state)
  {
  case sm_StartDisplay:
    for (unsigned char i = 0; i < 8; i++) // Displays the moving grid by default
    {
      Matrix_write(MAX7219_REG_DIGIT0 + i, gameGrid[i]);
    }
    break;
  case sm_GameDisplay:
    if (counter1 < 26)
    {
      Shifting('L');
      counter1++;
    }
    else if (counter < 45 && counter1 >= 25)
    {
      Shifting('R');
      counter++;
    }
    else if (RealCounterL < 45)
    {
      if (j < 8)
      {
        MovingGameGrid[j] = (MovingGameGrid[j] << 1) | 0b00000001; // Shift Left once and set far left bit
        Matrix_write(MAX7219_REG_DIGIT0 + j, MovingGameGrid[j]);
        j++;
      }
      else if (j >= 8)
      {
        j = 0;
      }
      RealCounterL++;
    }
    else if (RealCounterR < 45 && RealCounterL >= 45)
    {
      if (j < 8)
      {
        MovingGameGrid[j] = (MovingGameGrid[j] >> 1) | 0b10000000; // Shift right once and set far left bit
        Matrix_write(MAX7219_REG_DIGIT0 + j, MovingGameGrid[j]);
        j++;
      }
      else if (j >= 8)
      {
        j = 0;
      }
      RealCounterR++;
    }

    if (RealCounterL >= 45 && RealCounterR >= 45)
    {
      RealCounterL = 0;
      RealCounterR = 0;
    }
    updateCarPosition();

    break;
  case sm_ResetDisplay:
    counter1 = 0;
    counter = 0;
    RealCounterL = 0;
    RealCounterR = 0;
    FirstRunL = 0;
    FirstRunR = 0;
    currScore = -1;
    carPos = 4;
    j = 0;
    p = 0;
    ICR1 = 0;
    OCR1A = 0; // 50% duty cycle
    noteIndex = 0;
    deadIndex = 0;
    for (int k = 0; k < 8; k++) // Resets the movingGrid
    {
      MovingGameGrid[k] = ResetGrid[k];
    }
    for (int k = 0; k < 8; k++) // Resets the carGrid
    {
      carGrid[k] = ResetCarGrid[k];
    }
    GameState = 0;

    break;
  }
  return MDisplay_state;
}

//  ===========================================================================================================================================
//  ===========================================================================================================================================
//  ===========================================================================================================================================
//  ===========================================================================================================================================
//  ===========================================================================================================================================
//  ===========================================================================================================================================

// Function to check collision between carGrid and MovingGameGrid
int checkCollision(unsigned char carGrid[8], unsigned char MovingGameGrid[8])
{
  for (int i = 0; i < 8; ++i)
  {
    // Perform bitwise AND between corresponding rows of carGrid and MovingGameGrid
    if (carGrid[i] & MovingGameGrid[i])
    {
      // Collision detected if any bit is set in the result of the AND operation
      return 1;
    }
  }
  // No collision detected
  return 0;
}

int joystick_cnt = 0;
int Joystick_Tick(int Joystick_state)
{
  // serial_println(ADC_read(2));
  switch (Joystick_state)
  {
  case sm_Idle:
    if (ADC_read(2) >= 650 && carPos < 7)
    {
      // serial_println(ADC_read(2));
      Joystick_state = sm_Right;
    }
    else if (ADC_read(2) <= 400 && carPos > 0)
    {
      // serial_println(ADC_read(2));
      Joystick_state = sm_Left;
    }
    break;
  case sm_Left:
    if (ADC_read(2) >= 400)
    {
      Joystick_state = sm_Idle;
    }
    break;
  case sm_Right:
    if (ADC_read(2) <= 650)
    {
      Joystick_state = sm_Idle;
    }
    break;
  }

  // ACTIONSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
  switch (Joystick_state)
  {
  case sm_Idle:
    joystick_cnt = 0;
    break;

  case sm_Left:
    // Move the car left only once when transitioning to the left state
    if (joystick_cnt < 1)
    {
      carGrid[6] &= ~(1 << carPos); // Turn off the current bit
      carPos--;
      // serial_println(carPos);
      carGrid[6] |= (1 << carPos); // Turn on the bit to the left
      EndGame = checkCollision(carGrid, MovingGameGrid);
      joystick_cnt++;
    }
    break;

  case sm_Right:
    // Move the car right only once when transitioning to the right state
    if (joystick_cnt < 1)
    {
      carGrid[6] &= ~(1 << carPos); // Turn off the current bit
      carPos++;
      // serial_println(carPos);
      carGrid[6] |= (1 << carPos); // Turn on the bit to the right
      EndGame = checkCollision(carGrid, MovingGameGrid);
      joystick_cnt++;
    }
    break;
  }

  if (currScore > HighScore)
  {
    HighScore = currScore;
  }

  return Joystick_state;
}

//
//
//
//
//
//
//
//
//
// HELPER FUNCTIONS FOR LCD
//
int NumList[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
const char *CharNumList[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};

void LCDCurrScore(int currScore)
{
  if (currScore < 10)
  {
    lcd_goto_xy(0, 12);
    for (unsigned i = 0; i < 10; i++)
    {
      if (currScore == NumList[i])
      {
        lcd_write_character(*CharNumList[i]);
      }
    }
  }
  else if (currScore >= 10)
  {
    int Tenths = (currScore / 10) % 10;
    lcd_goto_xy(0, 12);
    lcd_write_character(' ');
    lcd_goto_xy(0, 12);

    for (unsigned j = 0; j < 10; j++)
    {
      if (Tenths == NumList[j])
      {
        lcd_write_character(*CharNumList[j]);
      }
    }
    lcd_goto_xy(0, 13);
    int Ones = currScore % 10;
    for (unsigned i = 0; i < 10; i++)
    {
      if (Ones == NumList[i])
      {
        lcd_write_character(*CharNumList[i]);
      }
    }
  }
}

void LCDHighScore(int HighScore)
{
  if (HighScore < 10)
  {
    lcd_goto_xy(1, 11);
    for (unsigned i = 0; i < 10; i++)
    {
      if (HighScore == NumList[i])
      {
        lcd_write_character(*CharNumList[i]);
      }
    }
  }
  else if (HighScore >= 10)
  {
    int Tenths = (HighScore / 10) % 10;
    lcd_goto_xy(1, 11);
    lcd_write_character(' ');
    lcd_goto_xy(1, 11);

    for (unsigned j = 0; j < 10; j++)
    {
      if (Tenths == NumList[j])
      {
        lcd_write_character(*CharNumList[j]);
      }
    }
    lcd_goto_xy(1, 12);
    int Ones = currScore % 10;
    for (unsigned i = 0; i < 10; i++)
    {
      if (Ones == NumList[i])
      {
        lcd_write_character(*CharNumList[i]);
      }
    }
  }
}
//
//

int LCD_Tick(int LCD_state)
{

  switch (LCD_state)
  {
  case sm_Display:
    break;
  }
  // Action State
  switch (LCD_state)
  {
  case sm_Display:
    // serial_println(Start);
    if (Start == 0)
    {
      lcd_goto_xy(0, 0);
      lcd_write_str("PRESS 1 TO START");
      Start++;
    }
    if (Restart >= 1)
    {
      Start = 0;
      Restart = 0;
      lcd_clear();
    }

    if (Start >= 2)
    {
      lcd_clear();
      lcd_goto_xy(0, 0);
      lcd_write_str("Curr Score: ");
      lcd_goto_xy(1, 0);
      lcd_write_str("HighScore: ");
      currScore++;
      LCDHighScore(HighScore);
      LCDCurrScore(currScore);

      GameState++;
    }
    if (ADC_read(0) > 800)
    {
      Start++;
    }
    if (ADC_read(1) > 800)
    {
      Restart++;
      // serial_println(Restart);
    }
    if (EndGame == 1)
    {
      GameState = 0;
      Start = -1;

      lcd_clear();
      lcd_goto_xy(1, 0);
      lcd_write_str("GAME OVER :((");
      lcd_goto_xy(0, 0);
      lcd_write_str("Your Score: ");
      lcd_goto_xy(0, 12);
      LCDCurrScore(currScore);
      for (int k = 0; k < 8; k++) // Resets the movingGrid
      {
        MovingGameGrid[k] = EndGameGrid[k];
      }
      EndGame++;
      // You may also want to reset other game variables if needed
    }
  }
  return LCD_state;
}

//
//
//
//

const int chromaticScale[] = {329, 349, 392, 392, 349, 329, 293, 261, 261, 293, 329, 329, 293, 261};

const int scaleLength = sizeof(chromaticScale) / sizeof(chromaticScale[0]);

void setupTimer1ForBuzzer(int frequencyIndex)
{
  int topValue = (16000000 / (8 * frequencyIndex)) - 1;

  ICR1 = topValue;

  OCR1A = topValue / 2;
}

void ENDGAMEMUSIC(int frequencyIndex)
{
  int topValue = (16000000 / (8 * frequencyIndex)) - 1;

  ICR1 = topValue;

  OCR1A = topValue / 2;
}
int Buzzer_Tick(int Buzzer_state)
{
  switch (Buzzer_state)
  {
  case sm_BuzzIdle:
    if (GameState >= 1)
    {
      Buzzer_state = sm_PlayNote;
    }
    else if (EndGame == 1)
    {
      Buzzer_state = sm_EndGameMusic;
    }
    break;
  case sm_PlayNote:
    if (Buzzercnt >= 250 && EndGame != 1)
    {
      Buzzer_state = sm_NextNote;
      Buzzercnt = 0;
    }
    else if (EndGame == 1)
    {
      Buzzer_state = sm_EndGameMusic;
      Buzzercnt = 0;
    }
    else if (ADC_read(1) > 800)
    {
      Buzzer_state = sm_BuzzIdle;
    }
    break;
  case sm_NextNote:
    if (Buzzercnt == 50 && EndGame != 1)
    {
      Buzzer_state = sm_PlayNote;
      Buzzercnt = 0;
      noteIndex++;
    }
    else if (ADC_read(1) > 800)
    {
      Buzzer_state = sm_BuzzIdle;
    }
    break;
  case sm_EndGameMusic:
    if (ADC_read(1) > 800)
    {
      Buzzer_state = sm_BuzzIdle;
    }
    break;
  }

  switch (Buzzer_state)
  {
  case sm_BuzzIdle:
    // Turn off the buzzer

    ICR1 = 0;
    OCR1A = 0;
    noteIndex = 0; // Reset noteIndex to start from the beginning of the melody
    deadIndex = 0;
    break;
  case sm_PlayNote:
    setupTimer1ForBuzzer(chromaticScale[noteIndex]);
    Buzzercnt++;
    break;
  case sm_NextNote:
    if (noteIndex > scaleLength)
    {
      noteIndex = 0;
    }

    Buzzercnt++;
    break;
  case sm_EndGameMusic:
    ENDGAMEMUSIC(293);
    break;
  }

  return Buzzer_state;
}

//
//
//
//
//
//
//
//
//
//

//  ===========================================================================================================================================

void TimerISR()
{
  for (unsigned int i = 0; i < NUM_TASKS; i++)
  { // Iterate through each task in the task array
    if (tasks[i].elapsedTime == tasks[i].period)
    {                                                    // Check if the task is ready to tick
      tasks[i].state = tasks[i].TickFct(tasks[i].state); // Tick and set the next state for this task
      tasks[i].elapsedTime = 0;                          // Reset the elapsed time for the next tick
    }
    tasks[i].elapsedTime += GCD_PERIOD; // Increment the elapsed time by GCD_PERIOD
  }
}
// ===========================================================================================================================================

int main(void)
{

  ADC_init();
  serial_init(9600);
  SPI_INIT();
  Matrix_init();
  lcd_init();
  lcd_clear(); // Clear the display

  DDRD = 0xFF;        //  OUTPUT
  DDRC = 0x00;        // INPUT
  DDRB |= (1 << PB1); // Set pin 9 (PB1/OC1A) as output

  TCCR1A = 0;
  TCCR1B = 0;

  // Set Timer1 to Fast PWM mode using ICR1 as TOP
  TCCR1A |= (1 << WGM11) | (1 << COM1A1);              // Fast PWM, clear OC1A on compare match, set at BOTTOM (non-inverting mode)
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM with ICR1 as TOP, prescaler 8

  ICR1 = 0; // 20ms pwm period

  OCR1A = 0;
  ;

  tasks[0].period = Matrix_Period;
  tasks[0].state = sm_StartDisplay;
  tasks[0].elapsedTime = 0;
  tasks[0].TickFct = &MatrixDisplay_Tick;

  tasks[1].period = Joyststick_Period;
  tasks[1].state = sm_Idle;
  tasks[1].elapsedTime = 0;
  tasks[1].TickFct = &Joystick_Tick;

  tasks[2].period = LCD_Period;
  tasks[2].state = sm_Display;
  tasks[2].elapsedTime = 0;
  tasks[2].TickFct = &LCD_Tick;

  tasks[3].period = Buzzer_Period;
  tasks[3].state = sm_BuzzIdle;
  tasks[3].elapsedTime = 0;
  tasks[3].TickFct = &Buzzer_Tick;

  // for (unsigned char i = 0; i < 8; i++) // Displays the moving grid by default
  // {
  //   Matrix_write(MAX7219_REG_DIGIT0 + i, gameGrid[i]);
  // }

  TimerSet(GCD_PERIOD);
  TimerOn();

  while (1)
  {
    // serial_println(ADC_read(2));
  }

  return 0;
}
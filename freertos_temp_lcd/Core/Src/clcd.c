#include     "main.h"

#include     "clcd.h"

void CLCD_send(int type, char data)

{

    if (type == DATA4)

        HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

    else

        HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (data & (1 << 7)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (data & (1 << 6)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (data & (1 << 5)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data & (1 << 4)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

    HAL_Delay(1);

    if (type != CMD8)

    {

        HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

        HAL_Delay(1);

        HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);

        HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (data & (1 << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (data & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (data & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_Delay(1);

        HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

        HAL_Delay(1);
    }
}

void CLCD_init()

{

       HAL_Delay(15);                                       // wait for 15ms from power-on

       CLCD_send(CMD8, FUNCTION_SET | BIT_8);         // Initial Value

       HAL_Delay(5);                                  // wait for 5ms from power-on

       CLCD_send(CMD8, FUNCTION_SET | BIT_8);         // Initial Value

       HAL_Delay(1);                                  // wait for 1ms from power-on

       CLCD_send(CMD8, FUNCTION_SET | BIT_8);         // Initial Value

       CLCD_send(CMD8, FUNCTION_SET | BIT_4);         // Function Set = 4 Bit Interface + 2 Line + 5x8 Dot

       CLCD_send(CMD4, FUNCTION_SET | BIT_4 | LINE_2 | DOT_58);    // Function Set = 4 Bit Interface + 2 Line + 5x8 Dot

       CLCD_send(CMD4, DISP_MODE_SET | DISP_OFF);           // Display Off

       CLCD_send(CMD4, CLEAR_DISPLAY);                // Clear Display, Curror Position = 0

       CLCD_send(CMD4, ENTRI_SET | CURSOR_INC | CURSOR_NOSHIFT);   // Entry Mode Set = Cursor Increment + Cursor No Shift

       CLCD_send(CMD4, DISP_MODE_SET | DISP_ON | CURSOR_OFF | BLINK_OFF); // Display Set = Display On + Cursor Off + Blink Off

}

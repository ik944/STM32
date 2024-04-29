#ifndef INC_CLCD_H_

#define INC_CLCD_H_

 

#define      CMD8                             0

#define      CMD4                             1

#define      DATA4                            2

 

#define      FUNCTION_SET               0x20   // Function Set(bit 5)

#define      BIT_8                            0x10   // 8 Bit Interface (bit 4)

#define      BIT_4                            0x00   // 4 Bit Interface (bit 4)

#define      LINE_2                           0x08   // 2 Line (bit 3)

#define      LINE_1                           0x00   // 1 Line (bit 3)

#define      DOT_511                                 0x04   // 5x8 Dot (bit 2)

#define      DOT_58                           0x00   // 5x8 Dot (bit 2)

#define      DISP_MODE_SET              0x08   // Set Display Mode

#define      DISP_ON                                 0x04   // Display On

#define      DISP_OFF                         0x00   // Display Off

#define      CURSOR_ON                        0x02   // Cursor On

#define      CURSOR_OFF                       0x00   // Cursor Off

#define      BLINK_ON                         0x01   // Blink On

#define      BLINK_OFF                        0x00   // Blink Off

#define      ENTRI_SET                        0x04   // Entry Mode Set (bit 2)

#define CURSOR_INC                      0x02   // Cursor Increment (bit 1)

#define CURSOR_NOINC                    0x00   // Cursor No Increment (bit 1)

#define CURSOR_SHIFT                    0x01   // Display Shift (bit 0)

#define CURSOR_NOSHIFT                  0x00   // Display No Shift (bit 0)

#define      CLEAR_DISPLAY              0x01   // Clear Display, Cursor Position = 0

 

#endif /* INC_CLCD_H_ */
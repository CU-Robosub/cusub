#ifndef	__TERMINAL_TEST_H__
#define	__TERMINAL_TEST_H__

struct termios *configure;

/************************************************************
Function that configures and opens the dev/tty port to be us-
ed by the application. Since this is just a test there is no 
return value. 
************************************************************/

void tty_config(struct termios *con, int descriptor);


#endif /*__TERMINAL_TEST_H__*/

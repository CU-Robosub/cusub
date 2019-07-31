#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include "terminal_test.h"
#include "checksum.h"
#include "circbuf.h"

#include <sstream>

#define POLL_RESP_LENGTH  (0x35)

CircBuf_t * Tx_Buf;
uint8_t set_data_to_retreive[] = {0x00, 0x0d, 0x03, 0x07, 0x4d, 0x15, 0x16, 0x17, 0x4a, 0x4b, 0x4c, 0x0d, 0x43};
uint8_t acquire_data[] = {0x00, 0x05, 0x04, 0xbf, 0x71};
char *device = "/dev/ttyUSB2";
int fd;
uint8_t c;
volatile uint8_t rec_data[53];

static uint16_t   crc_ccitt_generic( const unsigned char *input_str, size_t num_bytes, uint16_t start_value );
static void             init_crcccitt_tab( void );
static bool             crc_tabccitt_init       = false;
static uint16_t         crc_tabccitt[256];

uint16_t crc_xmodem( const unsigned char *input_str, size_t num_bytes )
{
  return crc_ccitt_generic( input_str, num_bytes, CRC_START_XMODEM );
}

static uint16_t crc_ccitt_generic( const unsigned char *input_str, size_t num_bytes, uint16_t start_value )
{
  uint16_t crc;
  uint16_t tmp;
  uint16_t short_c;
  const unsigned char *ptr;
  size_t a;

  if ( ! crc_tabccitt_init ) init_crcccitt_tab();

  crc = start_value;
  ptr = input_str;

  if ( ptr != NULL ) for (a=0; a<num_bytes; a++) {

    short_c = 0x00ff & (unsigned short) *ptr;
    tmp     = (crc >> 8) ^ short_c;
    crc     = (crc << 8) ^ crc_tabccitt[tmp];

    ptr++;
  }
  return crc;
}

static void init_crcccitt_tab( void )
{
  uint16_t i;
  uint16_t j;
  uint16_t crc;
  uint16_t c;

  for (i=0; i<256; i++) {

    crc = 0;
    c   = i << 8;

    for (j=0; j<8; j++) {

      if ( (crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ CRC_POLY_CCITT;
      else                      crc =   crc << 1;

      c = c << 1;
    }

    crc_tabccitt[i] = crc;
  }

  crc_tabccitt_init = true;
}

void tty_config(struct termios *con, int descriptor)
{
  tcgetattr(descriptor, con);
  con->c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  con->c_oflag = 0;
  con->c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  con->c_cc[VMIN]  = 1;
  con->c_cc[VTIME] = 0;
  if(cfsetispeed(con, B38400) || cfsetospeed(con, B38400))
  {
    perror("ERROR in baud set\n");
  }

  if(tcsetattr(descriptor, TCSAFLUSH, con) < 0)
  {
    perror("ERROR in set attr\n");
  }
}

void poll_for_data()
{
  write(fd, &set_data_to_retreive, sizeof(set_data_to_retreive));
  write(fd, &acquire_data, sizeof(acquire_data));

  size_t i = 0;
  uint8_t len = POLL_RESP_LENGTH;
  while(len > 0)
  {
    if(read(fd, &c, 1) > 0)
    {
      //printf("x%02x ", c);
      //fflush(stdout);
      // circbuf_add(Tx_Buf, c);
      rec_data[i] = c;
      i++;
      len--;
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pni_trax_imu_node");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/imu_raw", 1000);

  ros::Rate loop_rate(1000);

  //Tx_Buf = (CircBuf_t *)malloc(sizeof(CircBuf_t));  //allocate memory to transmit buffer
  //circbuf_init(Tx_Buf, 100);

  fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if(fd == -1)
  {
    perror("ERROR opening file descriptor\n");
  }

  // res  = crc_xmodem((const unsigned char *)&set_data_to_retreive, sizeof(set_data_to_retreive));

  // printf("%x\n", res);

  configure = (struct termios*)malloc(sizeof(struct termios));

  tty_config(configure, fd);

  float *b;
  uint32_t accel_x = 0, accel_y = 0, accel_z = 0, anglvel_x = 0, anglvel_y = 0, anglvel_z = 0;

  sensor_msgs::Imu msg = sensor_msgs::Imu();
  while (ros::ok())
  {



    poll_for_data();

    // msg.orientation.x = //call to function having 'x' data
    // msg.orientation.y = //call to function having 'y' data
    // msg.orientation.z = //call to function having 'z' data
    // msg.orientation.w = //call to function having 'w' data
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    msg.orientation_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
    msg.angular_velocity_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
    msg.linear_acceleration_covariance = {0.01,0,0,0,0.01,0,0,0,0.01};
    anglvel_x = (uint32_t)(rec_data[37]<<24) | (uint32_t)(rec_data[38]<<16) | (uint32_t)(rec_data[39]<<8) | (uint32_t)(rec_data[40]);
    b = (float *)&anglvel_x;
    msg.angular_velocity.x = (double)(*b);//call to function having 'x' data

    anglvel_y = (uint32_t)(rec_data[42]<<24) | (uint32_t)(rec_data[43]<<16) | (uint32_t)(rec_data[44]<<8) | (uint32_t)(rec_data[45]);
    b = (float *)&anglvel_y;
    msg.angular_velocity.y = (double)(*b);//call to function having 'y' data

    anglvel_z = (uint32_t)(rec_data[47]<<24) | (uint32_t)(rec_data[48]<<16) | (uint32_t)(rec_data[49]<<8) | (uint32_t)(rec_data[50]);
    b = (float *)&anglvel_z;
    msg.angular_velocity.z = (double)(*b);//call to function having 'z' data

    accel_x = (uint32_t)(rec_data[22]<<24) | (uint32_t)(rec_data[23]<<16) | (uint32_t)(rec_data[24]<<8) | (uint32_t)(rec_data[25]);
    b = (float *)&accel_x;
    msg.linear_acceleration.x = (9.80665)*((double)(*b));//call to function having 'x' data

    accel_y = (uint32_t)(rec_data[27]<<24) | (uint32_t)(rec_data[28]<<16) | (uint32_t)(rec_data[29]<<8) | (uint32_t)(rec_data[30]);
    b = (float *)&accel_y;
    msg.linear_acceleration.y = (9.80665)*((double)(*b));//call to function having 'y' data

    accel_z = (uint32_t)(rec_data[32]<<24) | (uint32_t)(rec_data[33]<<16) | (uint32_t)(rec_data[34]<<8) | (uint32_t)(rec_data[35]);
    b = (float *)&accel_z;
    msg.linear_acceleration.z = (9.80665)*((double)(*b));//call to function having 'z' data

    // ROS_INFO("%s", "publish imu messages");

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  close(fd);
  return 0;
}

/* ambi-tv: a flexible ambilight clone for embedded linux
*  Copyright (C) 2013 Georg Kaindl
*  
*  This file is part of ambi-tv.
*  
*  ambi-tv is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  
*  ambi-tv is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with ambi-tv.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "ws2801-spidev-sink.h"

#include "../util.h"
#include "../log.h"
#include "../color.h"

#define DEFAULT_DEV_NAME         "/dev/spidev0.0"
#define DEFAULT_SPI_SPEED        2500000
#define DEFAULT_GAMMA            1.6   // works well for me, but ymmv...

#define LOGNAME                  "ws2801-spidev: "

static const char ambitv_ws2801_spidev_mode  = 0;
static const char ambitv_ws2801_spidev_bits  = 8;
static const char ambitv_ws2801_spidev_lsbf  = 0;

struct ambitv_ws2801_priv {
   char*             device_name;
   int               fd, spi_speed, num_leds, actual_num_leds, grblen;
   int               led_len[4], *led_str[4];   // top, bottom, left, right
   double            led_inset[4];              // top, bottom, left, right
   unsigned char*    grb;
   unsigned char**   bbuf;
   int               num_bbuf, bbuf_idx;
   double            gamma[3];      // RGB gamma, not GRB!
   unsigned char*    gamma_lut[3];  // also RGB
};

static int*
ambitv_ws2801_ptr_for_output(struct ambitv_ws2801_priv* ws2801, int output, int* led_str_idx, int* led_idx)
{
   int idx = 0, *ptr = NULL;
   
   if (output < ws2801->num_leds) {
      while(output >= ws2801->led_len[idx]) {
         output -= ws2801->led_len[idx];
         idx++;
      }
      
      if (ws2801->led_str[idx][output] >= 0) {
         ptr = &ws2801->led_str[idx][output];
      
         if (led_str_idx)
            *led_str_idx = idx;
         
         if (led_idx)
            *led_idx = output;
      }
   }
   
   return ptr;
}

static int
ambitv_ws2801_map_output_to_point(
   struct ambitv_sink_component* component,
   int output,
   int width,
   int height,
   int* x,
   int* y)
{
   int ret = -1, *outp = NULL, str_idx = 0, led_idx = 0;
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   outp = ambitv_ws2801_ptr_for_output(ws2801, output, &str_idx, &led_idx);
   
   if (NULL != outp) {
      ret = 0;
      float llen  = ws2801->led_len[str_idx] - 1;
      float dim   = (str_idx < 2) ? width : height;
      float inset = ws2801->led_inset[str_idx] * dim;
      dim -= 2*inset;
      
      switch (str_idx) {
         case 0:  // top
            *x = (int)CONSTRAIN(inset + (dim / llen) * led_idx, 0, width);
            *y = 0;
            break;
         case 1:  // bottom
            *x = (int)CONSTRAIN(inset + (dim / llen) * led_idx, 0, width);
            *y = height;
            break;
         case 2:  // left
            *x = 0;
            *y = (int)CONSTRAIN(inset + (dim / llen) * led_idx, 0, height);
            break;
         case 3:  // right
            *x = width;
            *y = (int)CONSTRAIN(inset + (dim / llen) * led_idx, 0, height);
            break;
         default:
            ret = -1;
            break;
      }
   } else {
      *x = *y = -1;
   }
   
   return ret;
}

static int
ambitv_ws2801_commit_outputs(struct ambitv_sink_component* component)
{
   int ret = -1;
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
      
   if (ws2801->fd >= 0) {
      ret = write(ws2801->fd, ws2801->grb, ws2801->grblen);
      
      if (ret != ws2801->grblen) {
         if (ret <= 0)
            ret = -errno;
         else 
            ret = -ret;
      } else
         ret = 0;
   }
   
   if (ws2801->num_bbuf)
      ws2801->bbuf_idx = (ws2801->bbuf_idx + 1) % ws2801->num_bbuf;
   
   return ret;
}

static void
ambitv_ws2801_clear_leds(struct ambitv_sink_component* component)
{
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   if (NULL != ws2801->grb && ws2801->fd >= 0) {
      int i;
      
      // send 3 times, in case there's noise on the line,
      // so that all LEDs will definitely be off afterwards.
      for (i=0; i<3; i++) {
         memset(ws2801->grb, 0, ws2801->grblen);
         (void)ambitv_ws2801_commit_outputs(component);
      }
   }
}

static int
ambitv_ws2801_set_output_to_rgb(
   struct ambitv_sink_component* component,
   int idx,
   int r,
   int g,
   int b)
{
   int ret = -1, *outp = NULL, i, *rgb[] = {&r, &g, &b};
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   outp = ambitv_ws2801_ptr_for_output(ws2801, idx, NULL, NULL);
   
   if (NULL != outp) {
      int ii = *outp;
      
      if (ws2801->num_bbuf) {
         unsigned char* acc = ws2801->bbuf[ws2801->bbuf_idx];
         
         acc[3 * ii]             = b; // b;
         acc[3 * ii + 1]         = g; // g;
         acc[3 * ii + 2]         = r; // r;
         
         r = g = b = 0;
         for (i=0; i<ws2801->num_bbuf; i++) {
            b += ws2801->bbuf[i][3 * ii];
            g += ws2801->bbuf[i][3 * ii + 1];
            r += ws2801->bbuf[i][3 * ii + 2];
         }
         
         b /= ws2801->num_bbuf;
         g /= ws2801->num_bbuf;
         r /= ws2801->num_bbuf;
      }
      
      for (i=0; i<3; i++) {
         if (ws2801->gamma_lut[i])
            *rgb[i] = ambitv_color_map_with_lut(ws2801->gamma_lut[i], *rgb[i]);
      }
      
      ws2801->grb[3 * ii]       = b; //g
      ws2801->grb[3 * ii + 1]   = g; //r
      ws2801->grb[3 * ii + 2]   = r; //b
      
      ret = 0;
   }
   
   return ret;
}

static int
ambitv_ws2801_num_outputs(struct ambitv_sink_component* component)
{
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   return ws2801->num_leds;
}

static int
ambitv_ws2801_start(struct ambitv_sink_component* component)
{
   int ret = 0;
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   if (ws2801->fd < 0) {
      ws2801->fd = open(ws2801->device_name, O_WRONLY);
      if (ws2801->fd < 0) {
         ret = ws2801->fd;
         ambitv_log(ambitv_log_error, LOGNAME "failed to open device '%s' : %d (%s).\n",
            ws2801->device_name, errno, strerror(errno));
         goto errReturn; 
      }
      
      ret = ioctl(ws2801->fd, SPI_IOC_WR_MODE, &ambitv_ws2801_spidev_mode);
      if (ret < 0) {
         ambitv_log(ambitv_log_error, LOGNAME "failed to spidev mode on device '%s' : %d (%s).\n",
            ws2801->device_name, errno, strerror(errno));
         goto closeReturn;
      }
      
      ret = ioctl(ws2801->fd, SPI_IOC_WR_BITS_PER_WORD, &ambitv_ws2801_spidev_bits);
      if (ret < 0) {
         ambitv_log(ambitv_log_error, LOGNAME "failed to spidev bits-per-word on device '%s' : %d (%s).\n",
            ws2801->device_name, errno, strerror(errno));
         goto closeReturn;
      }
      
      ret = ioctl(ws2801->fd, SPI_IOC_WR_MAX_SPEED_HZ, &ws2801->spi_speed);
      if (ret < 0) {
         ambitv_log(ambitv_log_error, LOGNAME "failed to spidev baudrate on device '%s' : %d (%s).\n",
            ws2801->device_name, errno, strerror(errno));
         goto closeReturn;
      }
      
      ret = ioctl(ws2801->fd, SPI_IOC_WR_LSB_FIRST, &ambitv_ws2801_spidev_lsbf);
      if (ret < 0) {
         ambitv_log(ambitv_log_error, LOGNAME "failed to spidev bitorder on device '%s' : %d (%s).\n",
            ws2801->device_name, errno, strerror(errno));
         goto closeReturn;
      }
   }

   ambitv_ws2801_clear_leds(component);
   
   return ret;

closeReturn:
   close(ws2801->fd);
   ws2801->fd = -1;
errReturn:
   return ret;
}

static int
ambitv_ws2801_stop(struct ambitv_sink_component* component)
{
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   ambitv_ws2801_clear_leds(component);
   
   if (ws2801->fd >= 0) {
      close(ws2801->fd);
      ws2801->fd = -1;
   }
   
   return 0;
}

static int
ambitv_ws2801_configure(struct ambitv_sink_component* component, int argc, char** argv)
{
   int i, c, ret = 0;
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   memset(ws2801->led_str, 0, sizeof(int*) * 4);
   ws2801->num_leds = ws2801->actual_num_leds = 0;

      
   if (NULL == ws2801)
      return -1;

   static struct option lopts[] = {
      { "spi-device", required_argument, 0, 'd' },
      { "spi-speed-hz", required_argument, 0, 'h' },
      { "leds-top", required_argument, 0, '0' },
      { "leds-bottom", required_argument, 0, '1' },
      { "leds-left", required_argument, 0, '2' },
      { "leds-right", required_argument, 0, '3' },
      { "blended-frames", required_argument, 0, 'b' },
      { "gamma-red", required_argument, 0, '4'},
      { "gamma-green", required_argument, 0, '5'},
      { "gamma-blue", required_argument, 0, '6'},
      { "led-inset-top", required_argument, 0, 'w'},
      { "led-inset-bottom", required_argument, 0, 'x'},
      { "led-inset-left", required_argument, 0, 'y'},
      { "led-inset-right", required_argument, 0, 'z'},
      { NULL, 0, 0, 0 }
   };
   
   while (1) {    
      c = getopt_long(argc, argv, "", lopts, NULL);

      if (c < 0)
         break;

      switch (c) {
         case 'd': {            
            if (NULL != optarg) {
               if (NULL != ws2801->device_name)
                  free(ws2801->device_name);

               ws2801->device_name = strdup(optarg);
            }
            break;
         }

         case 'h': {
            if (NULL != optarg) {
               char* eptr = NULL;
               long nbuf = strtol(optarg, &eptr, 10);

               if ('\0' == *eptr && nbuf > 0) {
                  ws2801->spi_speed = (int)nbuf;
               } else {
                  ambitv_log(ambitv_log_error, LOGNAME "invalid argument for '%s': '%s'.\n",
                     argv[optind-2], optarg);
                  ret = -1;
                  goto errReturn;
               }
            }

            break;
         }
         
         case 'b': {
            if (NULL != optarg) {
               char* eptr = NULL;
               long nbuf = strtol(optarg, &eptr, 10);
         
               if ('\0' == *eptr && nbuf >= 0) {
                  ws2801->num_bbuf = (int)nbuf;
               } else {
                  ambitv_log(ambitv_log_error, LOGNAME "invalid argument for '%s': '%s'.\n",
                     argv[optind-2], optarg);
                  ret = -1;
                  goto errReturn;
               }
            }
         
            break;
         }
         
         case '0':
         case '1':
         case '2':
         case '3': {
            int idx = c - '0';
                        
            ret = ambitv_parse_led_string(optarg, &ws2801->led_str[idx], &ws2801->led_len[idx]);
            if (ret < 0) {
               ambitv_log(ambitv_log_error, LOGNAME "invalid led configuration string for '%s': '%s'.\n",
                  argv[optind-2], optarg);
               goto errReturn;
            }
            
            ws2801->num_leds += ws2801->led_len[idx];
            for (i=0; i<ws2801->led_len[idx]; i++)
               if (ws2801->led_str[idx][i] >= 0)
                  ws2801->actual_num_leds++;
            
            break;
         }
         
         case '4':
         case '5':
         case '6': {
            if (NULL != optarg) {
               char* eptr = NULL;
               double nbuf = strtod(optarg, &eptr);
         
               if ('\0' == *eptr) {
                  ws2801->gamma[c-'4'] = nbuf;
               } else {
                  ambitv_log(ambitv_log_error, LOGNAME "invalid argument for '%s': '%s'.\n",
                     argv[optind-2], optarg);
                  ret = -1;
                  goto errReturn;
               }
            }
         
            break;
         }
         
         case 'w':
         case 'x':
         case 'y':
         case 'z': {
            if (NULL != optarg) {
               char* eptr = NULL;
               double nbuf = strtod(optarg, &eptr);
         
               if ('\0' == *eptr) {
                  ws2801->led_inset[c-'w'] = nbuf / 100.0;
               } else {
                  ambitv_log(ambitv_log_error, LOGNAME "invalid argument for '%s': '%s'.\n",
                     argv[optind-2], optarg);
                  ret = -1;
                  goto errReturn;
               }
            }
         
            break;
         }

         default:
            break;
      }
   }

   if (optind < argc) {
      ambitv_log(ambitv_log_error, LOGNAME "extraneous argument: '%s'.\n",
         argv[optind]);
      ret = -1;
   }

errReturn:
   return ret;
}

static void
ambitv_ws2801_print_configuration(struct ambitv_sink_component* component)
{
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;

   ambitv_log(ambitv_log_info,
      "\tdevice name:       %s\n"
      "\tspi hz:            %d\n"
      "\tnumber of leds:    %d\n"
      "\tblending frames:   %d\n"
      "\tled insets (tblr): %.1f%%, %.1f%%, %.1f%%, %.1f%%\n"
      "\tgamma (rgb):       %.2f, %.2f, %.2f\n",
         ws2801->device_name,
         ws2801->spi_speed,
         ws2801->actual_num_leds,
         ws2801->num_bbuf,
         ws2801->led_inset[0]*100.0, ws2801->led_inset[1]*100.0,
         ws2801->led_inset[2]*100.0, ws2801->led_inset[3]*100.0,
         ws2801->gamma[0], ws2801->gamma[1], ws2801->gamma[2]
   );
}

void
ambitv_ws2801_free(struct ambitv_sink_component* component)
{
   int i;
   struct ambitv_ws2801_priv* ws2801 =
      (struct ambitv_ws2801_priv*)component->priv;
   
   if (NULL != ws2801) {
      if (NULL != ws2801->device_name)
         free(ws2801->device_name);
      
      if (NULL != ws2801->grb)
         free(ws2801->grb);
      
      if (NULL != ws2801->bbuf) {
         for (i=0; i<ws2801->num_bbuf; i++)
            free(ws2801->bbuf[i]);
         free(ws2801->bbuf);
      }
      
      for (i=0; i<3; i++) {
         if (NULL != ws2801->gamma_lut[i])
            ambitv_color_gamma_lookup_table_free(ws2801->gamma_lut[i]);
      }
      
      free(ws2801);
   }
}

struct ambitv_sink_component*
ambitv_ws2801_create(const char* name, int argc, char** argv)
{  
   struct ambitv_sink_component* ws2801 =
      ambitv_sink_component_create(name);
   
   if (NULL != ws2801) {
      int i;
      struct ambitv_ws2801_priv* priv =
         (struct ambitv_ws2801_priv*)malloc(sizeof(struct ambitv_ws2801_priv));
      
      ws2801->priv = priv;
      
      memset(priv, 0, sizeof(struct ambitv_ws2801_priv));
      
      priv->fd             = -1;
      priv->spi_speed      = DEFAULT_SPI_SPEED;
      priv->device_name    = strdup(DEFAULT_DEV_NAME);
      priv->gamma[0]       = DEFAULT_GAMMA;
      priv->gamma[1]       = DEFAULT_GAMMA;
      priv->gamma[2]       = DEFAULT_GAMMA;
      
      ws2801->f_print_configuration   = ambitv_ws2801_print_configuration;
      ws2801->f_start_sink            = ambitv_ws2801_start;
      ws2801->f_stop_sink             = ambitv_ws2801_stop;
      ws2801->f_num_outputs           = ambitv_ws2801_num_outputs;
      ws2801->f_set_output_to_rgb     = ambitv_ws2801_set_output_to_rgb;
      ws2801->f_map_output_to_point   = ambitv_ws2801_map_output_to_point;
      ws2801->f_commit_outputs        = ambitv_ws2801_commit_outputs;
      ws2801->f_free_priv             = ambitv_ws2801_free;
      
      if (ambitv_ws2801_configure(ws2801, argc, argv) < 0)
         goto errReturn;
      
      priv->grblen   = sizeof(unsigned char) * 3 * priv->actual_num_leds;
      priv->grb      = (unsigned char*)malloc(priv->grblen);
      
      if (priv->num_bbuf > 1) {
         priv->bbuf = (unsigned char**)malloc(sizeof(unsigned char*) * priv->num_bbuf);
         for (i=0; i<priv->num_bbuf; i++) {
            priv->bbuf[i] = (unsigned char*)malloc(priv->grblen);
            memset(priv->bbuf[i], 0, priv->grblen);
         }
      } else
         priv->num_bbuf = 0;

      memset(priv->grb, 0, priv->grblen);
      
      for (i=0; i<3; i++) {
         if (priv->gamma[i] >= 0.0) {
            priv->gamma_lut[i] =
               ambitv_color_gamma_lookup_table_create(priv->gamma[i]);
         }
      }
	  
   }
   
   return ws2801;

errReturn:
   ambitv_sink_component_free(ws2801);
   return NULL;
}

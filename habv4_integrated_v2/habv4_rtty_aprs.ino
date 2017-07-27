//---APRS----------------------------------------------------------------------------------------------

void send_APRS() {
  ax25_init();
  tx_aprs();
}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2B */
  pinMode(HX1_TXD, OUTPUT);
  //===Modified for Mega================================================
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  //===Modified for Mega================================================
  
}

void tx_aprs()
{
  aprstxstatus=1;

  //===Modified for Mega================================================
  PORTH |= _BV(3); //_BV(HX1_ENABLE); // Same as digitalWrite(HX1_ENABLE, HIGH); but more efficient, digitalWrite(HX1_ENABLE, HIGH); //
  //===Modified for Mega================================================
  
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
     * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  int32_t aprs_alt = alt * 32808 / 10000;


  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);
  if(tempsensors==1)
  {
    ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,%i'C,project zephyrus",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus,temperature1
      );
  }
  else
  {
    ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,%i,%i'C,project zephyrus",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus,temperature1,temperature2
      );
  }
  seq++;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v) {
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...) {
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */

  //===Modified for Mega================================================
  TIMSK2 |= _BV(TOIE2); //0x01; 
  //===Modified for Mega================================================
  
  //PORTA |= TXENABLE;
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid) {
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

//---RTTY----------------------------------------------------------------------------------------------

void setMTX2Frequency() {
  //Modify to change RTTY transmission
  float _mtx2comp;
  int _mtx2int;
  long _mtx2fractional;
  char _mtx2command[17];
  MTX2_EN.begin(9600);
  _mtx2comp=(MTX2_FREQ+0.0015)/6.5;
  _mtx2int=_mtx2comp;
  _mtx2fractional=float(((_mtx2comp-_mtx2int)+1)*524288);
  snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
  delay(100);
  MTX2_EN.print(_mtx2command);
  delay(50);
  MTX2_EN.end();
}  

uint16_t crccat(char *msg) {
  uint16_t x;
  while(*msg == '$') msg++;
  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);
  snprintf_P(msg, 8, PSTR("*%04X\n"), x);
  return(x);
}

void initialise_interrupt() {
  //Initialize TIMER1 for RTTY: MODIFIED TO TIMER4 FOR MEGA
  cli();          // disable global interrupts

  //===Modified for Mega================================================
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM22);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS20);
  TCCR1B |= (1 << CS22);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  //===Modified for Mega================================================

  sei();          // enable global interrupts
}

void rtty_txbit (int bit) {
  if (bit)
  {
    digitalWrite(MTX2_TXD,HIGH);
    //#ifdef APRS
//    analogWrite(MTX2_TXD, MTX2_OFFSET+((MTX2_SHIFT*4)/16)); // High
    //#endif
    //#ifndef APRS
    //    analogWrite(MTX2_TXD, MTX2_OFFSET+((MTX2_SHIFT*1.8)/16)); // High
    //#endif

  }
  else
  {
      digitalWrite(MTX2_TXD,LOW);
//    analogWrite(MTX2_TXD, MTX2_OFFSET); // Low
  }
}

//---Interrupt handlers----------------------------------------------------------------------------------------------

//===Modified for Mega================================================
ISR(TIMER1_COMPA_vect) {
//===Modified for Mega================================================

  //Intrerrupt handler for RTTY on TIMER1
  if(alt>1000 && sats >= 4)
  {
    digitalWrite(LED_WARN,LOW);  
    digitalWrite(LED_OK,LOW);  
  }
  else 
  {
    currentMillis = millis();
    if(currentMillis - previousMillis > ONE_SECOND) 
    {
      previousMillis = currentMillis;   
      if(errorstatus!=8)
      {
        digitalWrite(LED_WARN,!digitalRead(LED_WARN));
        digitalWrite(LED_OK,LOW);  
      }
      else 
      {
        digitalWrite(LED_OK, !digitalRead(LED_OK)); 
        digitalWrite(LED_WARN,LOW);    
      }
    }
  }

  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD)) { 
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
    lockvariables=1;
    #ifndef APRS
        if(tempsensors==1)
        {
          snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,errorstatus);
        }
        else
        {
          snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,temperature2,battvaverage,errorstatus);
        }
    #endif
    #ifdef APRS
        if(tempsensors==1)
        {
          snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,battvaverage,aprs_attempts,errorstatus);
        }
        else
        {
          snprintf(txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%i,%i,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,temperature1,temperature2,battvaverage,aprs_attempts,errorstatus);
        }
    #endif
    crccat(txstring);
    maxalt=0;
    lockvariables=0;
    txstringlength=strlen(txstring);
    txstatus=2;
    txj=0;
    break;
  case 2: // Grab a char and lets go transmit it. 
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else 
    {
      txstatus=0; // Should be finished
      txj=0;
      count++;
    }
    break;
  case 3:
    if(txi<ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1); 
      else rtty_txbit(0);  
      txc = txc >> 1;
      break;
    }
    else 
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    } 
  case 4:
    if(STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }

  }
}

//===Modified for Mega================================================
ISR(TIMER2_OVF_vect) {
  //Intrerrupt on TIMER2 for APRS: Modified to TIMER3 for Arduino Mega PWM control of pin3
//===Modified for Mega================================================
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  /* Update the PWM output */
  
  //===Modified for Mega================================================
  OCR2A = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  //===Modified for Mega================================================
  
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */

        //digitalWrite(HX1_ENABLE, LOW);

        //===Modified for Mega================================================
        PORTH &= ~_BV(3); // Turn the HX1 Off
        //===Modified for Mega================================================
        
        aprstxstatus=0;
        
        //===Modified for Mega================================================
        TIMSK2 &= ~_BV(TOIE2); //0x01;//
        //===Modified for Mega================================================

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

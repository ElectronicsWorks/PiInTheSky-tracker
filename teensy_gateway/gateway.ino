/*
 * LoRa gateway code ported from Dave Ackerman's PITS lora-gateway source - just the radio and packet
 * decode parts.  It has support for dual radios and uplink capability but that isn't currently used
 * by the overall program.
 *
 * Based on version V1.8.7
 */

// RFM98
uint8_t currentMode = 0x81;

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
#define REG_CURRENT_RSSI            0x1B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define REG_DETECTION_THRESHOLD     0x37

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04

// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23    // 0010 0011
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0xC0    // 1100 0000


struct TLoRaDevice
{
  double Frequency;
  double Bandwidth;
  double CurrentBandwidth;

  int InUse;
  int DIO0;
  int DIO5;
  double activeFreq;

  int AFC;          // Enable Automatic Frequency Control
  double MaxAFCStep;      // Maximum adjustment, in kHz, per packet
  int AFCTimeout;       // Revert to original frequency if no packets for this period (in seconds)

  int SpeedMode;
  int Power;
  int PayloadLength;
  int ImplicitOrExplicit;
  int ErrorCoding;
  int SpreadingFactor;
  int LowDataRateOptimize;
  unsigned int TelemetryCount, SSDVCount, BadCRCCount, UnknownCount;
  int Sending;
  uint32_t LastSSDVPacketAt, LastTelemetryPacketAt;
  uint32_t ReturnToCallingModeAt;
  uint32_t ReturnToOriginalFrequencyAt;
  int InCallingMode;
  int ActivityLED;
  double UplinkFrequency;
  int UplinkMode;

  // Normal (non TDM) uplink
  int UplinkTime;
  int UplinkCycle;
};


struct TConfig
{
  int EnableTelemetryLogging;
  int EnablePacketLogging;
  int CallingTimeout;
  struct TLoRaDevice LoRaDevices[2];
};


// ???DJJ Added Mode 7 (from existing pits mode 7) and new mode 8
struct TLoRaMode
{
  int ImplicitOrExplicit;
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
  int BaudRate;
  const char *Description;
} LoRaModes[] =
{
  {EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_20K8, SPREADING_11, 1,    60, "Telemetry"},   // 0: Normal mode for telemetry
  {IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_6,  0,  1400, "SSDV"},        // 1: Normal mode for SSDV
  {EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_62K5, SPREADING_8,  0,  2000, "Repeater"},    // 2: Normal mode for repeater network
  {EXPLICIT_MODE, ERROR_CODING_4_6, BANDWIDTH_250K, SPREADING_7,  0,  8000, "Turbo"},       // 3: Normal mode for high speed images in 868MHz band
  {IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_250K, SPREADING_6,  0, 16828, "TurboX"},      // 4: Fastest mode within IR2030 in 868MHz band
  {EXPLICIT_MODE, ERROR_CODING_4_8, BANDWIDTH_41K7, SPREADING_11, 0,   200, "Calling"},     // 5: Calling mode
  {IMPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_41K7, SPREADING_6,  0,  2800, "Uplink"},      // 6: Uplink mode for 868
  {EXPLICIT_MODE, ERROR_CODING_4_5, BANDWIDTH_20K8, SPREADING_7,  0,  2800, "Telnet"},      // 7: Telnet-style comms with HAB on 434
  {EXPLICIT_MODE, ERROR_CODING_4_6, BANDWIDTH_62K5, SPREADING_8,  0,  1200, "SSDV-US"}      // 8: Experimental mode for SSDV in US (915 MHz)
};

struct TConfig Config;

struct TBandwidth
{
  int LoRaValue;
  double Bandwidth;
  const char *ConfigString;
} Bandwidths[] =
{
  {BANDWIDTH_7K8,     7.8,  "7K8"},
  {BANDWIDTH_10K4,   10.4,  "10K4"},
  {BANDWIDTH_15K6,   15.6,  "15K6"},
  {BANDWIDTH_20K8,   20.8,  "20K8"},
  {BANDWIDTH_31K25,  31.25, "31K25"},
  {BANDWIDTH_41K7,   41.7,  "41K7"},
  {BANDWIDTH_62K5,   62.5,  "62K5"},
  {BANDWIDTH_125K,  125.0,  "125K"},
  {BANDWIDTH_250K,  250.0,  "250K"},
  {BANDWIDTH_500K,  500.0,  "500K"}
};


char *ftoa (float f, int places) {
    // i is the integer portion
    unsigned int i = floor(abs(f));
    int digits = 0;
    static char rv[32];
    static char fv[32];
    if (f < 0.0) {
        rv[0] = '-';
        itoa(i, &rv[1], 10);
    } else {
        itoa(i, rv, 10);
    }
    strcat(rv, ".");
 
    // turn fraction into whole number
    float r = abs(f) - (float)i;
    i = 0;
    while (r > 0.0f) {
        r *= 10;
        i += (int)r;
        r -= (int)r;
    // do not use a "places" greater than 8 or i
    // could wrap around!
    // (this is enough to capture the precision of 
    // a float anyway)
        if (++digits == places) break;
        i *= 10;
    }
    strcat(rv, itoa(i, fv, 10));
    return rv;
};


void
writeRegister( int Channel, uint8_t reg, uint8_t val )
{
  unsigned char data[2];

  data[0] = reg | 0x80;
  data[1] = val;
  wiringPiSPIDataRW( Channel, data, 2 );
}

uint8_t
readRegister( int Channel, uint8_t reg )
{
  unsigned char data[2];
  uint8_t val;

  data[0] = reg & 0x7F;
  data[1] = 0;
  wiringPiSPIDataRW( Channel, data, 2 );
  val = data[1];

  return val;
}


void LogPacket( int Channel, int8_t SNR, int RSSI, double FreqError, int Bytes, unsigned char MessageType )
{
  if ( Config.EnablePacketLogging ) {
    char s[80];

    sprintf(s,
            "%lu - Ch %d, SNR %d, RSSI %d, FreqErr %s, Bytes %d, Type %02Xh\n",
            SecCount, Channel, SNR, RSSI, ftoa(FreqError, 1), Bytes, MessageType );
    LogString(s);
  }
}


void LogTelemetryPacket(char *Telemetry)
{
  if (Config.EnableTelemetryLogging) {
    char s[128];

    sprintf(s, "%lu - %s\n", SecCount, Telemetry );
    LogString(s);
  }
}

void LogMessage( const char *format, ... )
{
  char buffer[128];
  va_list args;

  va_start( args, format );
  vsprintf( buffer, format, args );
  va_end( args );

  LogString(buffer);
}

void ChannelPrintf(int Channel, int row, int column, const char *format, ... )
{
  char Buffer[80];
  va_list args;

  if (Channel == 0) {
    va_start( args, format );
    vsprintf( Buffer, format, args );
    va_end( args );
    DispString(row, column, Buffer);
  }
}

void
setMode( int Channel, uint8_t newMode )
{
  if ( newMode == currentMode )
    return;

  switch ( newMode )
  {
    case RF98_MODE_TX:
      writeRegister( Channel, REG_LNA, LNA_OFF_GAIN );    // TURN LNA OFF FOR TRANSMITT
      writeRegister( Channel, REG_PA_CONFIG, Config.LoRaDevices[Channel].Power );
      writeRegister( Channel, REG_OPMODE, newMode );
      currentMode = newMode;
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister( Channel, REG_PA_CONFIG, PA_OFF_BOOST );  // TURN PA OFF FOR RECIEVE??
      writeRegister( Channel, REG_LNA, LNA_MAX_GAIN );    // MAX GAIN FOR RECEIVE
      writeRegister( Channel, REG_OPMODE, newMode );
      currentMode = newMode;
      // LogMessage("Changing to Receive Continuous Mode\n");
      break;
    case RF98_MODE_SLEEP:
      writeRegister( Channel, REG_OPMODE, newMode );
      currentMode = newMode;
      // LogMessage("Changing to Sleep Mode\n");
      break;
    case RF98_MODE_STANDBY:
      writeRegister( Channel, REG_OPMODE, newMode );
      currentMode = newMode;
      // LogMessage("Changing to Standby Mode\n");
      break;
    default:
      return;
  }

  if ( newMode != RF98_MODE_SLEEP )
  {
    while ( digitalRead( Config.LoRaDevices[Channel].DIO5 ) == 0 )
    {
    }
    // delay(1);
  }

  LogMessage("Mode Change Done\n");
  return;
}

void setFrequency( int Channel, double Frequency )
{
  unsigned long FrequencyValue;

  //FrequencyValue = ( unsigned long ) ( Frequency * 7110656 / 434 );
  FrequencyValue = ( unsigned long ) ( Frequency * 16384 );

  writeRegister( Channel, 0x06, ( FrequencyValue >> 16 ) & 0xFF );    // Set frequency
  writeRegister( Channel, 0x07, ( FrequencyValue >> 8 ) & 0xFF );
  writeRegister( Channel, 0x08, FrequencyValue & 0xFF );

  Config.LoRaDevices[Channel].activeFreq = Frequency;

  LogMessage("Set Frequency to %s\n", ftoa(Frequency, 1));

  ChannelPrintf( Channel, 5, 0, "%s MHz ", ftoa(Frequency, 4) );
}

void displayFrequency ( int Channel, double Frequency )
{
  ChannelPrintf( Channel, 5, 0, "%s MHz ", ftoa(Frequency, 4) );
}

void setLoRaMode( int Channel )
{
  LogMessage("Setting LoRa Mode\n");
  setMode( Channel, RF98_MODE_SLEEP );
  writeRegister( Channel, REG_OPMODE, 0x80 );

  setMode( Channel, RF98_MODE_SLEEP );

  LogMessage("Set Default Frequency\n");
  setFrequency( Channel, Config.LoRaDevices[Channel].Frequency);
}

int IntToSF(int Value)
{
  return Value << 4;
}

int SFToInt(int SpreadingFactor)
{
  return SpreadingFactor >> 4;
}

int IntToEC(int Value)
{
  return (Value - 4) << 1;
}

int ECToInt(int ErrorCoding)
{
  return (ErrorCoding >> 1) + 4;
}

int DoubleToBandwidth(double Bandwidth)
{
  int i;

  for (i = 0; i < 10; i++)
  {
    if (abs(Bandwidth - Bandwidths[i].Bandwidth) < (Bandwidths[i].Bandwidth / 10))
    {
      return Bandwidths[i].LoRaValue;
    }
  }

  return BANDWIDTH_20K8;
}

double BandwidthToDouble(int LoRaValue)
{
  int i;

  for (i = 0; i < 10; i++)
  {
    if (LoRaValue == Bandwidths[i].LoRaValue)
    {
      return Bandwidths[i].Bandwidth;
    }
  }

  return 20.8;
}

int IntToLowOpt(int Value)
{
  return Value ? 0x08 : 0;
}

int LowOptToInt(int LowOpt)
{
  return LowOpt ? 1 : 0;
}

void SetLoRaParameters( int Channel, int ImplicitOrExplicit, int ErrorCoding, double Bandwidth, int SpreadingFactor, int LowDataRateOptimize )
{
  writeRegister( Channel, REG_MODEM_CONFIG, ImplicitOrExplicit | IntToEC(ErrorCoding) | DoubleToBandwidth(Bandwidth));
  writeRegister( Channel, REG_MODEM_CONFIG2, IntToSF(SpreadingFactor) | CRC_ON );
  writeRegister( Channel, REG_MODEM_CONFIG3, 0x04 | IntToLowOpt(LowDataRateOptimize));    // 0x04: AGC sets LNA gain
  writeRegister( Channel, REG_DETECT_OPT, ( readRegister( Channel, REG_DETECT_OPT ) & 0xF8 ) | ( ( SpreadingFactor == 6 ) ? 0x05 : 0x03 ) );    // 0x05 For SF6; 0x03 otherwise
  writeRegister( Channel, REG_DETECTION_THRESHOLD, ( SpreadingFactor == 6 ) ? 0x0C : 0x0A );    // 0x0C for SF6, 0x0A otherwise

  Config.LoRaDevices[Channel].CurrentBandwidth = Bandwidth;      // Used for AFC - current bandwidth may be different to that configured (i.e. because we're using calling mode)
}

void SetDefaultLoRaParameters( int Channel )
{
  LogMessage("Set Default Parameters\n");

  SetLoRaParameters( Channel,
                     Config.LoRaDevices[Channel].ImplicitOrExplicit,
                     Config.LoRaDevices[Channel].ErrorCoding,
                     Config.LoRaDevices[Channel].Bandwidth,
                     Config.LoRaDevices[Channel].SpreadingFactor,
                     Config.LoRaDevices[Channel].LowDataRateOptimize );
}


/////////////////////////////////////
//    Method:   Setup to receive continuously
//////////////////////////////////////
void
startReceiving( int Channel )
{
  writeRegister( Channel, REG_DIO_MAPPING_1, 0x00 );  // 00 00 00 00 maps DIO0 to RxDone

  writeRegister( Channel, REG_PAYLOAD_LENGTH, 255 );
  writeRegister( Channel, REG_RX_NB_BYTES, 255 );

  writeRegister( Channel, REG_FIFO_RX_BASE_AD, 0 );
  writeRegister( Channel, REG_FIFO_ADDR_PTR, 0 );

  // Setup Receive Continous Mode
  setMode( Channel, RF98_MODE_RX_CONTINUOUS );
}

void ReTune( int Channel, double FreqShift )
{
  setMode( Channel, RF98_MODE_SLEEP );
  LogMessage( "Retune by %skHz\n", ftoa(FreqShift * 1000, 1) );
  setFrequency( Channel, Config.LoRaDevices[Channel].activeFreq + FreqShift );
  startReceiving( Channel );
}

void SendLoRaData(int Channel, char *buffer, int Length)
{
  unsigned char data[257];
  int i;

  // Change frequency for the uplink ?
  if (Config.LoRaDevices[Channel].UplinkFrequency > 0)
  {
    LogMessage("Change frequency to %sMHz\n", ftoa(Config.LoRaDevices[Channel].UplinkFrequency, 3));
    setFrequency(Channel, Config.LoRaDevices[Channel].UplinkFrequency);
  }

  // Change mode for the uplink ?
  if (Config.LoRaDevices[Channel].UplinkMode >= 0)
  {
    int UplinkMode;

    UplinkMode = Config.LoRaDevices[Channel].UplinkMode;

    LogMessage("Change LoRa mode to %d\n", Config.LoRaDevices[Channel].UplinkMode);

    SetLoRaParameters(Channel,
                      LoRaModes[UplinkMode].ImplicitOrExplicit,
                      ECToInt(LoRaModes[UplinkMode].ErrorCoding),
                      BandwidthToDouble(LoRaModes[UplinkMode].Bandwidth),
                      SFToInt(LoRaModes[UplinkMode].SpreadingFactor),
                      0);
  }

  LogMessage( "LoRa Channel %d Sending %d bytes\n", Channel, Length );
  Config.LoRaDevices[Channel].Sending = 1;

  setMode( Channel, RF98_MODE_STANDBY );

  writeRegister( Channel, REG_DIO_MAPPING_1, 0x40 );  // 01 00 00 00 maps DIO0 to TxDone

  writeRegister( Channel, REG_FIFO_TX_BASE_AD, 0x00 );    // Update the address ptr to the current tx base address
  writeRegister( Channel, REG_FIFO_ADDR_PTR, 0x00 );

  data[0] = REG_FIFO | 0x80;
  for ( i = 0; i < Length; i++ )
  {
    data[i + 1] = buffer[i];
  }
  wiringPiSPIDataRW( Channel, data, Length + 1 );

  // Set the length. For implicit mode, since the length needs to match what the receiver expects, we have to set a value which is 255 for an SSDV packet
  writeRegister( Channel, REG_PAYLOAD_LENGTH, Length );

  // go into transmit mode
  setMode( Channel, RF98_MODE_TX );
}

void ShowPacketCounts(int Channel)
{
  if (Config.LoRaDevices[Channel].InUse)
  {
    ChannelPrintf( Channel, 3, 0, "Telem: %d (%us)",
                   Config.LoRaDevices[Channel].TelemetryCount,
                   Config.LoRaDevices[Channel].LastTelemetryPacketAt ? (unsigned int) (SecCount - Config.LoRaDevices[Channel].LastTelemetryPacketAt) : 0);
    ChannelPrintf( Channel, 4, 0, "Image: %d (%us)",
                   Config.LoRaDevices[Channel].SSDVCount,
                   Config.LoRaDevices[Channel].
                   LastSSDVPacketAt ? ( unsigned int ) ( SecCount - Config.LoRaDevices[Channel].LastSSDVPacketAt ) : 0 );
  }
}

void ProcessUploadMessage(int Channel, char *Message)
{
  LogMessage("Ch %d: Uploaded message %s\n", Channel, Message);
}

void ProcessCallingMessage(int Channel, char *Message)
{
  char Payload[16];
  double Frequency;
  int ImplicitOrExplicit, ErrorCoding, Bandwidth, SpreadingFactor, LowDataRateOptimize;

  if ( sscanf( Message + 2, "%15[^,],%lf,%d,%d,%d,%d,%d",
               Payload,
               &Frequency,
               &ImplicitOrExplicit,
               &ErrorCoding,
               &Bandwidth, &SpreadingFactor, &LowDataRateOptimize ) == 7 )
  {
    if (Config.LoRaDevices[Channel].AFC)
    {
      Frequency += (Config.LoRaDevices[Channel].activeFreq - Config.LoRaDevices[Channel].Frequency);
    }

    LogMessage( "Ch %d: Calling message, new frequency %s\n", Channel, ftoa(Frequency, 3) );

    // Decoded OK
    setMode( Channel, RF98_MODE_SLEEP );

    // setFrequency(Channel, Config.LoRaDevices[Channel].activeFreq + );
    setFrequency( Channel, Frequency );

    SetLoRaParameters( Channel, ImplicitOrExplicit, ECToInt(ErrorCoding), BandwidthToDouble(Bandwidth), SFToInt(SpreadingFactor), LowOptToInt(LowDataRateOptimize));

    setMode( Channel, RF98_MODE_RX_CONTINUOUS );

    Config.LoRaDevices[Channel].InCallingMode = 1;
  }
}

void ProcessLine(int Channel, char *Line)
{
  char Payload[16];
  unsigned int Counter;
  char Time[16];
  char Latitude[16];
  char Longitude[16];
  unsigned int Altitude;
  unsigned int dummy1;
  unsigned int dummy2;
  unsigned int numSats;
  char Temp[16];

  // Bail if we may have a bum line
  int n = strlen(Line);
  //LogMessage("n = %d, %c\n", n, Line[n-5]);
  if ((n < 60) || (Line[n-5] != '*')) {
    return;
  }

  // Parse key fields from sentence
  sscanf( Line + 2, "%15[^,],%u,%8[^,],%10[^,],%10[^,],%u,%u,%u,%u,%8[^*]",
          Payload,
          &Counter,
          Time,
          Latitude,
          Longitude,
          &Altitude,
          &dummy1,
          &dummy2,
          &numSats,
          Temp);

  // Update display
  ChannelPrintf(Channel, 0, 0, "%s, %s", Latitude, Longitude);
  ChannelPrintf(Channel, 1, 0, "%u M, %u F", Altitude, round(Altitude * 3.28084));
  ChannelPrintf(Channel, 2, 0, "%s C, %d F", Temp, round(atof(Temp)*1.8 + 32.0));
}

void ProcessTelemetryMessage(int Channel, char *Message)
{
  if (strlen(Message + 1) < 250)
  {
    char *startmessage, *endmessage;

    startmessage = Message;
    endmessage = strchr( startmessage, '\n' );

    while ( endmessage != NULL )
    {
      int Repeated;

      *endmessage = '\0';

      LogTelemetryPacket(startmessage);

      if ((Repeated = (*startmessage == '%'))) {
        *startmessage = '$';
      }

      ProcessLine(Channel, startmessage);

      Config.LoRaDevices[Channel].TelemetryCount++;

      startmessage = endmessage + 1;
      endmessage = strchr( startmessage, '\n' );
    }

    Config.LoRaDevices[Channel].LastTelemetryPacketAt = SecCount;
  }
}

static char *decode_callsign( char *callsign, uint32_t code )
{
  char *c, s;

  *callsign = '\0';

  /* Is callsign valid? */
  if ( code > 0xF423FFFF )
    return ( callsign );

  for ( c = callsign; code; c++ )
  {
    s = code % 40;
    if ( s == 0 )
      *c = '-';
    else if ( s < 11 )
      *c = '0' + s - 1;
    else if ( s < 14 )
      *c = '-';
    else
      *c = 'A' + s - 14;
    code /= 40;
  }
  *c = '\0';

  return ( callsign );
}

void ProcessSSDVMessage( int Channel, char *Message, int Repeated)
{
  // SSDV packet
  uint32_t CallsignCode;
  char Callsign[7];
  int ImageNumber, PacketNumber;

  Message[0] = 0x55;

  CallsignCode = Message[2];
  CallsignCode <<= 8;
  CallsignCode |= Message[3];
  CallsignCode <<= 8;
  CallsignCode |= Message[4];
  CallsignCode <<= 8;
  CallsignCode |= Message[5];

  decode_callsign( Callsign, CallsignCode );

  ImageNumber = Message[6];
  PacketNumber = Message[7] * 256 + Message[8];

  LogMessage("%lu Ch%d: SSDV Packet, Callsign %s, Image %d, Packet %d%s\n", SecCount, Channel, Callsign, ImageNumber, PacketNumber, Repeated ? " (repeated)" : "");

  Config.LoRaDevices[Channel].SSDVCount++;
  Config.LoRaDevices[Channel].LastSSDVPacketAt = SecCount;
}


int FixRSSI(int Channel, int RawRSSI, int SNR)
{
  int RSSI;

  if (Config.LoRaDevices[Channel].Frequency > 525)
  {
    // HF port (band 1)
    RSSI = RawRSSI - 157;
  }
  else
  {
    // LF port (Bands 2/3)
    RSSI = RawRSSI - 164;
  }

  if (SNR < 0)
  {
    RSSI += SNR / 4;
  }

  return RSSI;
}

int CurrentRSSI(int Channel)
{
  return FixRSSI(Channel, readRegister(Channel, REG_CURRENT_RSSI), 0);
}

int PacketSNR(int Channel)
{
  int8_t SNR;

  SNR = readRegister(Channel, REG_PACKET_SNR);
  SNR /= 4;

  return (int)SNR;
}

int PacketRSSI(int Channel)
{
  int SNR;

  SNR = PacketSNR(Channel);

  return FixRSSI(Channel, readRegister(Channel, REG_PACKET_RSSI), SNR);
}

void DIO0_Interrupt( int Channel )
{
  if ( Config.LoRaDevices[Channel].Sending )
  {
    Config.LoRaDevices[Channel].Sending = 0;
    // LogMessage( "Ch%d: End of Tx\n", Channel );

    setLoRaMode( Channel );
    SetDefaultLoRaParameters( Channel );
    startReceiving( Channel );
  }
  else
  {
    int Bytes;
    char Message[257];

    Bytes = receiveMessage( Channel, Message + 1 );

    if ( Bytes > 0 )
    {
      if ( Config.LoRaDevices[Channel].ActivityLED >= 0 )
      {
        digitalWrite( Config.LoRaDevices[Channel].ActivityLED, 1 );
        LEDCounts[Channel] = 5;
      }

      if ( Message[1] == '!' )
      {
        ProcessUploadMessage( Channel, Message + 1 );
      }
      else if ( Message[1] == '^' )
      {
        ProcessCallingMessage( Channel, Message + 1 );
      }
      else if ((Message[1] == '$') || (Message[1] == '%'))
      {
        ProcessTelemetryMessage(Channel, Message + 1);
      }
      else if ( Message[1] == '>' )
      {
        LogMessage( "Flight Controller message %d bytes = %s\n", Bytes, Message + 1 );
      }
      else if ( Message[1] == '*' )
      {
        LogMessage( "Uplink Command message %d bytes = %s\n", Bytes, Message + 1 );
      }
      else if (((Message[1] & 0x7F) == 0x66) ||   // SSDV JPG format
               ((Message[1] & 0x7F) == 0x67) ||   // SSDV other formats
               ((Message[1] & 0x7F) == 0x68) ||
               ((Message[1] & 0x7F) == 0x69))
      {
        int Repeated;

        // Handle repeater bit
        Repeated = Message[1] & 0x80;
        Message[1] &= 0x7F;

        ProcessSSDVMessage( Channel, Message, Repeated);
      }
      else
      {
        LogMessage("Unknown packet type is %02Xh, RSSI %d\n", Message[1], PacketRSSI(Channel));

        Config.LoRaDevices[Channel].UnknownCount++;
      }

      // Config.LoRaDevices[Channel].LastPacketAt = LastPacketAt;

      if (Config.LoRaDevices[Channel].InCallingMode && (Config.CallingTimeout > 0))
      {
        Config.LoRaDevices[Channel].ReturnToCallingModeAt = SecCount + Config.CallingTimeout;
      }

      if (!Config.LoRaDevices[Channel].InCallingMode && (Config.LoRaDevices[Channel].AFCTimeout > 0))
      {
        Config.LoRaDevices[Channel].ReturnToOriginalFrequencyAt = SecCount + Config.LoRaDevices[Channel].AFCTimeout;
      }

      ShowPacketCounts( Channel );
    }
  }
}

void DIO_Ignore_Interrupt_0( void )
{
  // nothing, obviously!
}

void
DIO0_Interrupt_0( void )
{
  DIO0_Interrupt( 0 );
}

void
DIO0_Interrupt_1( void )
{
  DIO0_Interrupt( 1 );
}

void setupRFM98( int Channel )
{
  if ( Config.LoRaDevices[Channel].InUse )
  {
    // initialize the pins
    pinMode( Config.LoRaDevices[Channel].DIO0, INPUT );
    pinMode( Config.LoRaDevices[Channel].DIO5, INPUT );

    wiringPiISR( Config.LoRaDevices[Channel].DIO0, RISING,
                 Channel > 0 ? &DIO0_Interrupt_1 : &DIO0_Interrupt_0 );

    wiringPiSPISetup( Channel, 500000, 0 );

    // LoRa mode
    setLoRaMode( Channel );

    SetDefaultLoRaParameters( Channel );

    startReceiving( Channel );
  }
}

double FrequencyError( int Channel )
{
  int32_t Temp;

  Temp = ( int32_t ) readRegister( Channel, REG_FREQ_ERROR ) & 7;
  Temp <<= 8L;
  Temp += ( int32_t ) readRegister( Channel, REG_FREQ_ERROR + 1 );
  Temp <<= 8L;
  Temp += ( int32_t ) readRegister( Channel, REG_FREQ_ERROR + 2 );

  if ( readRegister( Channel, REG_FREQ_ERROR ) & 8 )
  {
    Temp = Temp - 524288;
  }

  return -( ( double ) Temp * ( 1 << 24 ) / 32000000.0 ) * (Config.LoRaDevices[Channel].CurrentBandwidth / 500.0);
}

int receiveMessage( int Channel, char *message )
{
  int i, Bytes, currentAddr, x;
  unsigned char data[257];
  double FreqError;

  Bytes = 0;

  x = readRegister( Channel, REG_IRQ_FLAGS );
  // LogMessage("Message status = %02Xh\n", x);

  // clear the rxDone flag
  writeRegister( Channel, REG_IRQ_FLAGS, 0x40 );

  // check for payload crc issues (0x20 is the bit we are looking for
  if ( ( x & 0x20 ) == 0x20 )
  {
    LogMessage( "Ch%d: CRC Failure, RSSI %d\n", Channel, PacketRSSI(Channel));

    // reset the crc flags
    writeRegister( Channel, REG_IRQ_FLAGS, 0x20 );
    Config.LoRaDevices[Channel].BadCRCCount++;
    ShowPacketCounts( Channel );
  }
  else
  {
    currentAddr = readRegister( Channel, REG_FIFO_RX_CURRENT_ADDR );
    Bytes = readRegister( Channel, REG_RX_NB_BYTES );

    ChannelPrintf( Channel, 6, 0, "SNR: %d, RSSI: %d", PacketSNR(Channel), PacketRSSI(Channel));

    FreqError = FrequencyError( Channel ) / 1000;
    ChannelPrintf( Channel, 7, 0, "FERR: %skHz ", ftoa(FreqError, 1));
    ChannelPrintf( Channel, 7, 18, "%s",
                   Config.LoRaDevices[Channel].AFC ? "AFC" : "   " );

    writeRegister( Channel, REG_FIFO_ADDR_PTR, currentAddr );

    data[0] = REG_FIFO;
    wiringPiSPIDataRW( Channel, data, Bytes + 1 );
    for ( i = 0; i <= Bytes; i++ )
    {
      message[i] = data[i + 1];
    }

    message[Bytes] = '\0';

    LogPacket(Channel, PacketSNR(Channel), PacketRSSI(Channel), FreqError, Bytes, message[1]);

    if (Config.LoRaDevices[Channel].AFC && (fabs( FreqError ) > 0.5))
    {
      if (Config.LoRaDevices[Channel].MaxAFCStep > 0)
      {
        // Limit step to MaxAFCStep
        if (FreqError > Config.LoRaDevices[Channel].MaxAFCStep)
        {
          FreqError = Config.LoRaDevices[Channel].MaxAFCStep;
        }
        else if (FreqError < -Config.LoRaDevices[Channel].MaxAFCStep)
        {
          FreqError = -Config.LoRaDevices[Channel].MaxAFCStep;
        }
      }
      ReTune( Channel, FreqError / 1000 );
    }
  }

  // Clear all flags
  writeRegister( Channel, REG_IRQ_FLAGS, 0xFF );

  return Bytes;
}

void RemoveTrailingSlash(char *Value)
{
  int Len;

  if ((Len = strlen(Value)) > 0)
  {
    if ((Value[Len - 1] == '/') || (Value[Len - 1] == '\\'))
    {
      Value[Len - 1] = '\0';
    }
  }
}

void LoadConfigFile(void)
{
  int Channel;

  // Default configuration
  Config.LoRaDevices[0].InUse = CH0_ENABLE;
  Config.LoRaDevices[1].InUse = CH1_ENABLE;
  Config.CallingTimeout = CALLING_TIMEOUT;
  Config.LoRaDevices[0].ActivityLED = LED0;
  Config.LoRaDevices[1].ActivityLED = LED1;

  // Default pin allocations
  Config.LoRaDevices[0].DIO0 = DIO0_0;
  Config.LoRaDevices[0].DIO5 = DIO0_5;
  Config.LoRaDevices[1].DIO0 = DIO1_0;
  Config.LoRaDevices[1].DIO5 = DIO1_5;

  Config.LoRaDevices[0].Frequency = LORA0_FREQ;
  Config.LoRaDevices[1].Frequency = LORA1_FREQ;
  Config.LoRaDevices[0].SpeedMode = LORA0_SPEED_MODE;
  Config.LoRaDevices[1].SpeedMode = LORA1_SPEED_MODE;

  Config.EnableTelemetryLogging = ENABLE_TELEMETRY_LOGGING;
  Config.EnablePacketLogging = ENABLE_PACKET_LOGGING;

  for (Channel = 0; Channel <= 1; Channel++)
  {
    if ((Config.LoRaDevices[Channel].SpeedMode < 0) || (Config.LoRaDevices[Channel].SpeedMode >= (int) sizeof(LoRaModes) / (int) sizeof(LoRaModes[0]))) Config.LoRaDevices[Channel].SpeedMode = 0;

    // Defaults for this LoRa Mode
    Config.LoRaDevices[Channel].ImplicitOrExplicit = LoRaModes[Config.LoRaDevices[Channel].SpeedMode].ImplicitOrExplicit;
    Config.LoRaDevices[Channel].ErrorCoding = ECToInt(LoRaModes[Config.LoRaDevices[Channel].SpeedMode].ErrorCoding);
    Config.LoRaDevices[Channel].Bandwidth = BandwidthToDouble(LoRaModes[Config.LoRaDevices[Channel].SpeedMode].Bandwidth);
    Config.LoRaDevices[Channel].SpreadingFactor = SFToInt(LoRaModes[Config.LoRaDevices[Channel].SpeedMode].SpreadingFactor);
    Config.LoRaDevices[Channel].LowDataRateOptimize = LowOptToInt(LoRaModes[Config.LoRaDevices[Channel].SpeedMode].LowDataRateOptimize);
    Config.LoRaDevices[Channel].AFC = 1;
    Config.LoRaDevices[Channel].Power = PA_MAX_BOOST;
    Config.LoRaDevices[Channel].UplinkMode = -1;
    Config.LoRaDevices[Channel].MaxAFCStep = 0;
    Config.LoRaDevices[Channel].AFCTimeout = 0;

    // Clear any flags left over from a previous run
    writeRegister( Channel, REG_IRQ_FLAGS, 0xFF );
  }
}


void ProcessKeyPress( int ch )
{
  int Channel = 0;

  /* shifted keys act on channel 1 */
  if ( ch >= 'A' && ch <= 'Z' )
  {
    Channel = 1;
    /* change from upper to lower case */
    ch += ( 'a' - 'A' );
  }

  /* ignore if channel is not in use */
  if ( !Config.LoRaDevices[Channel].InUse && ch != 'h' )
  {
    return;
  }

  switch ( ch )
  {
    case 'f':
      Config.LoRaDevices[Channel].AFC =
        !Config.LoRaDevices[Channel].AFC;
      ChannelPrintf( Channel, 7, 18, "%s",
                     Config.LoRaDevices[Channel].AFC ? "AFC" : "   " );
      break;
    case 'a':
      ReTune( Channel, 0.1 );
      break;
    case 'z':
      ReTune( Channel, -0.1 );
      break;
    case 's':
      ReTune( Channel, 0.01 );
      break;
    case 'x':
      ReTune( Channel, -0.01 );
      break;
    case 'd':
      ReTune( Channel, 0.001 );
      break;
    case 'c':
      ReTune( Channel, -0.001 );
      break;
    case 'p':
      break;
    case 'h':
      LogConstString("Help:\n");
      LogConstString("  f - Toggle AFC\n");
      LogConstString("  a - Channel up 0.1 MHz\n");
      LogConstString("  z - Channel down 0.1 MHz\n");
      LogConstString("  s - Channel up 0.01 MHz\n");
      LogConstString("  x - Channel down 0.01 MHz\n");
      LogConstString("  d - Channel up 0.001 MHz\n");
      LogConstString("  c - Channel up 0.001 MHz\n");
      LogConstString("  h - help\n");
      break;
    default:
      // LogMessage("KeyPress %d\n", ch);
      return;
  }
}

void displayChannel (int Channel) {

  displayFrequency ( Channel, Config.LoRaDevices[Channel].Frequency );

  if (Config.LoRaDevices[Channel].AFC)
    ChannelPrintf( Channel, 7, 18, "AFC" );
  else
    ChannelPrintf( Channel, 7, 18, "   " );

}


void RadioMain() {
  int Channel;

  if (LoopPeriod > 1000) {
    // Every 1 second
    LoopPeriod = 0;
    SecCount++;

    for (Channel = 0; Channel <= 1; Channel++) {
      if ( Config.LoRaDevices[Channel].InUse ) {
        // Display some info
        ShowPacketCounts( Channel );
        ChannelPrintf( Channel, 5, 17, "%4d", CurrentRSSI(Channel));

        // Calling mode timeout?
        if ( Config.LoRaDevices[Channel].InCallingMode
             && ( Config.CallingTimeout > 0 )
             && ( Config.LoRaDevices[Channel].ReturnToCallingModeAt > 0 )
             && ( SecCount > Config.LoRaDevices[Channel].ReturnToCallingModeAt ) )
        {
          Config.LoRaDevices[Channel].InCallingMode = 0;
          Config.LoRaDevices[Channel].ReturnToCallingModeAt = 0;
          LogMessage( "Return to calling mode\n" );
          setLoRaMode( Channel );
          SetDefaultLoRaParameters( Channel );
          setMode( Channel, RF98_MODE_RX_CONTINUOUS );
        }

        // AFC Timeout ?
        if (!Config.LoRaDevices[Channel].InCallingMode &&
            (Config.LoRaDevices[Channel].AFCTimeout > 0) &&
            (Config.LoRaDevices[Channel].ReturnToOriginalFrequencyAt > 0) &&
            (SecCount > Config.LoRaDevices[Channel].ReturnToOriginalFrequencyAt))
        {
          Config.LoRaDevices[Channel].ReturnToOriginalFrequencyAt = 0;
          LogMessage("AFC timeout - return to original frequency\n");
          setMode(Channel, RF98_MODE_SLEEP);
          setFrequency(Channel, Config.LoRaDevices[Channel].Frequency);
          startReceiving(Channel);
        }

        // LEDs
        if (LEDCounts[Channel] && ( Config.LoRaDevices[Channel].ActivityLED >= 0))
        {
          if ( --LEDCounts[Channel] == 0 )
          {
            digitalWrite(Config.LoRaDevices[Channel].ActivityLED, 0);
          }
        }
      }
    }
  }
}



#include "CCodecSomfyRTS.h"

// To store pulse length, in microseconds
volatile word pulse;

// Interrupt handler
#if defined(__AVR_ATmega1280__)
    void ext_int_1(void) {
#else
    ISR(ANALOG_COMP_vect) {
#endif
    static word last;

    // determine the pulse length in microseconds, for either polarity
    pulse = micros() - last;
    last += pulse;
}

// Constructor : Serial
CCodecSomfyRTS::CCodecSomfyRTS(byte PORT_TX)
: _status(k_waiting_synchro)
, _cpt_synchro_hw(0)
, _cpt_bits(0)
, _previous_bit(0)
, _waiting_half_symbol(false)
{
  for(int i=0; i<7; ++i) _payload[i] = 0;

  _PORT_TX = PORT_TX; // Ex: A0
  _PORT_RX = 2;
}


void CCodecSomfyRTS::init(byte s, byte act_RX){
    if (s){
        Serial.begin(115200);
    }
    Serial.println("\n[SomfyDecoder]");

    pinMode(_PORT_TX, OUTPUT);
    digitalWrite(_PORT_TX, 0);

    if (act_RX){
        #if !defined(__AVR_ATmega1280__)
            pinMode(_PORT_RX, INPUT);  // use the AIO pin
            digitalWrite(_PORT_RX, 1); // enable pull-up

            // use analog comparator to switch at 1.1V bandgap transition
            ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);

            // set ADC mux to the proper port
            ADCSRA &= ~ bit(ADEN);
            ADCSRB |= bit(ACME);
            ADMUX = _PORT_RX - 1;
        #else
           attachInterrupt(1, ext_int_1, CHANGE);

           DDRE  &= ~_BV(PE5);
           PORTE &= ~_BV(PE5);
        #endif
    }
    Serial.println("Fin setup somfy_V2_Em");
}

CCodecSomfyRTS::t_status CCodecSomfyRTS::_CheckPulse(word p) {
  switch(_status) {
    case k_waiting_synchro:
      if (p > k_tempo_synchro_hw_min && p < k_tempo_synchro_hw_max) {
        ++_cpt_synchro_hw;
      }
      else if (p > k_tempo_synchro_sw_min && p < k_tempo_synchro_sw_max && _cpt_synchro_hw >= 4) {
        _cpt_bits = 0;
        _previous_bit = 0;
        _waiting_half_symbol = false;
        for(int i=0; i<7; ++i) _payload[i] = 0;
        _status = k_receiving_data;
      } else {
        _cpt_synchro_hw = 0;
      }
      break;

    case k_receiving_data:
      if (p > k_tempo_symbol_min && p < k_tempo_symbol_max && !_waiting_half_symbol) {
        _previous_bit = 1 - _previous_bit;
        _payload[_cpt_bits/8] += _previous_bit << (7 - _cpt_bits%8);
        ++_cpt_bits;
      } else if (p > k_tempo_half_symbol_min && p < k_tempo_half_symbol_max) {
        if (_waiting_half_symbol) {
          _waiting_half_symbol = false;
          _payload[_cpt_bits/8] += _previous_bit << (7 - _cpt_bits%8);
          ++_cpt_bits;
        } else {
          _waiting_half_symbol = true;
        }
      } else {
        _cpt_synchro_hw = 0;
        _status = k_waiting_synchro;
      }
      break;

    default:
      Serial.println("Internal error !");
      break;
  }

  t_status retval = _status;

  if (_status == k_receiving_data && _cpt_bits == 56) {
    retval = k_complete;
    decode();
    _status = k_waiting_synchro;
  }

  return retval;
}

bool CCodecSomfyRTS::decode() {
  // Dé-obfuscation
  byte frame[7];
  frame[0] = _payload[0];
  for(int i = 1; i < 7; ++i) frame[i] = _payload[i] ^ _payload[i-1];

 for(int i = 0; i < 7; ++i) Serial.print(frame[i], HEX); Serial.print(" ");
  Serial.println("");

  // Verification du checksum
  byte cksum = 0;
  for(int i = 0; i < 7; ++i) cksum = cksum ^ frame[i] ^ (frame[i] >> 4);
  cksum = cksum & 0x0F;
  if (cksum != 0) Serial.println("Checksum incorrect !");

  // Touche de controle
  switch(frame[1] & 0xF0) {
    case 0x10: Serial.println("My"); break;
    case 0x20: Serial.println("Up"); break;
    case 0x40: Serial.println("Down"); break;
    case 0x80: Serial.println("Prog"); break;
    case 0xF0: Serial.println("Gate"); break;
    default: Serial.println("???"); break;
  }
  Serial.print("Command: "); Serial.println(frame[1] & 0xF0, HEX);

  // Rolling code
  unsigned long rolling_code = (frame[2] << 8) + frame[3];
  Serial.print("Rolling code: "); Serial.println(rolling_code);

  // Adresse
  unsigned long address = ((unsigned long)frame[4] << 16) + (frame[5] << 8) + frame[6];
  Serial.print("Adresse: "); Serial.println(address, HEX);Serial.println("");
}
// ***********************************************************************************
bool CCodecSomfyRTS::transmit(byte cmd, byte first) {

  Serial.print("cmd= ");Serial.println(cmd,HEX);
  Serial.print("rc hex = ");Serial.println(_rolling_code,HEX);
  // Construction de la trame claire
  byte data[7];
  // Byte 0 : A7 = shutter (default) - A4 = Gate
  // Check Left value of command
  if (((cmd & 0xF0) >> 4) == 1){ data[0] = 0xA4; } else { data[0] = 0xA7; }
  data[1] = cmd << 4;
  data[2] = (_rolling_code & 0xFF00) >> 8;
  data[3] = _rolling_code & 0x00FF;
  // Set Specific Address
  data[4] = _remotecontrol[0];
  data[5] = _remotecontrol[1];
  data[6] = _remotecontrol[2];

  Serial.print("adr= ");Serial.print (data[4],HEX);Serial.print (data[5],HEX);Serial.println (data[6],HEX);

  for(int i = 0; i < 7; ++i) Serial.print(data[i],HEX);
  Serial.println("");

  // Calcul du checksum
  byte cksum = 0;
   for(int i = 0; i < 7; ++i) cksum = cksum ^ (data[i]&0xF) ^ (data[i] >> 4);  // ****************
  data[1] = data[1] | (cksum);  // *************************

   for(int i = 0; i < 7; ++i) Serial.print(data[i],HEX);
  Serial.println("");

  // Obsufscation *****************************
  byte datax[7];
  datax[0]=data[0];
  for(int i = 1; i < 7; ++i) datax[i] = datax[i-1] ^ data[i];  // ********************

   for(int i = 0; i < 7; ++i) Serial.print(datax[i],HEX);
  Serial.println("");

  // Emission wakeup, synchro hardware et software
  digitalWrite(_PORT_TX, 1);
  delayMicroseconds(k_tempo_wakeup_pulse);
  digitalWrite(_PORT_TX, 0);
  delayMicroseconds(k_tempo_wakeup_silence);

  for(int i=0; i<first; ++i) {
    digitalWrite(_PORT_TX, 1);
    delayMicroseconds(k_tempo_synchro_hw);
    digitalWrite(_PORT_TX, 0);
    delayMicroseconds(k_tempo_synchro_hw);
  }

  digitalWrite(_PORT_TX, 1);
  delayMicroseconds(k_tempo_synchro_sw);
  digitalWrite(_PORT_TX, 0);
  delayMicroseconds(k_tempo_half_symbol);

  // Emission des donnees
  for(int i=0; i<56;++i) {
    byte bit_to_transmit = (datax[i/8] >> (7 - i%8)) & 0x01;
    if (bit_to_transmit == 0) {
      digitalWrite(_PORT_TX, 1);
      delayMicroseconds(k_tempo_half_symbol);
      digitalWrite(_PORT_TX, 0);
      delayMicroseconds(k_tempo_half_symbol);
    }
    else
    {
      digitalWrite(_PORT_TX, 0);
      delayMicroseconds(k_tempo_half_symbol);
      digitalWrite(_PORT_TX, 1);
      delayMicroseconds(k_tempo_half_symbol);
    }
  }

  digitalWrite(_PORT_TX, 0);
  delayMicroseconds(k_tempo_inter_frame_gap);
}

void CCodecSomfyRTS::_RefreshRollingCode() {
    // Get Rolling Code in EEprom
    Serial.print("EEprom Adress Start : ");Serial.print(_remotecontrol[3]);
    byte lowByte  = EEPROM.read(_remotecontrol[3]);
    byte highByte = EEPROM.read(_remotecontrol[3] + 1);

    // Increment Rolling Code
    _rolling_code = (((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00)) + 1;

    // Set Rolling Code in EEprom
    lowByte = ((_rolling_code >> 0) & 0xFF);
    highByte = ((_rolling_code >> 8) & 0xFF);
    EEPROM.write(_remotecontrol[3], lowByte);
    EEPROM.write(_remotecontrol[3] + 1, highByte);
}

void CCodecSomfyRTS::Up(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x02, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x02, 7);
        }

        Serial.println("Up");
}

void CCodecSomfyRTS::Down(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x04, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x04, 7);
        }

        Serial.println("Down");
}

void CCodecSomfyRTS::MyStop(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();
        
        transmit(0x01, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x01, 7);
        }

        Serial.println("MyStop");
}


void CCodecSomfyRTS::UpStop(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x03, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x03, 7);
        }

        Serial.println("MyStop");
}

void CCodecSomfyRTS::DownStop(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x05, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x05, 7);
        }

        Serial.println("MyStop");
}

void CCodecSomfyRTS::Gate(RemoteControl remotecontrol,byte button) {

        _remotecontrol = remotecontrol;
        if (button){
            _remotecontrol[0] = remotecontrol[0] + 1;
            _remotecontrol[3] = remotecontrol[3] + 2;
        }
        _RefreshRollingCode();

        transmit(0x1F, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x1F, 7);
        }

        if (button){
            _remotecontrol[0] = remotecontrol[0] - 1;
            _remotecontrol[3] = remotecontrol[3] - 2;
        }

        Serial.println("Gate");
}

void CCodecSomfyRTS::AddProg(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x08, 2);
        int pmax=20;
        for(int p=0; p<pmax;++p) {
            transmit(0x08, 7);
        }

        Serial.println("Set Prog");
}

void CCodecSomfyRTS::RemoveProg(RemoteControl remotecontrol) {
        _remotecontrol = remotecontrol;
        _RefreshRollingCode();

        transmit(0x08, 2);
        int pmax=2;
        for(int p=0; p<pmax;++p) {
            transmit(0x08, 7);
        }

        Serial.println("Delete Prog");
}

void CCodecSomfyRTS::GetRC() {
      cli();
      word p = pulse;
      pulse = 0;
      sei();

      if (p != 0) {
        _CheckPulse(p);
      }
}

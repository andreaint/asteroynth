
#include "DaisyDuino.h"
#include "utility/gpio.h"

DaisyHardware hw;

#define LED1 D2
#define LED2 D0
#define LED3 D27
#define LED4 D1
#define LED5 D3
#define LED6 D26

#define KNOB2 A8
#define KNOB3 A9
#define KNOB4 A10
#define KNOB5 A11
#define KNOB6 A4
#define KNOB7 A5
#define KNOB8 A6
#define KNOB9 A7
#define KNOB10 A0
#define KNOB11 A1
#define KNOB12 A2
#define KNOB13 A3

#define JACKOUT1 D12
#define JACKOUT2 D13
#define JACKOUT3 D29
#define JACKOUT4 D14
#define JACKOUT5 D5

#define JACKIN1 D30
#define JACKIN2 D11
#define JACKIN3 D6
#define JACKIN4 D7
#define JACKIN5 D8
#define JACKIN6 D9
#define JACKIN7 D10
#define JACKIN8 D4

#define ARPMIN 5
#define ARPMAX 80
#define ARPRANGE (ARPMAX - ARPMIN)

#define ARPMIN2 40
#define ARPMAX2 70

long int knob1ValAccum = 0;
int knob1ValAccumIt = 0;

float knob1Val = 0;
float knob2Val = 0;
float knob3Val = 0;
float knob4Val = 0;
float knob5Val = 0;
float knob6Val = 0;
float knob7Val = 0;
float knob8Val = 0;
float knob9Val = 0;
float knob10Val = 0;
float knob11Val = 0;
float knob12Val = 0;
float knob13Val = 0;

long int timeLed1 = 0;
long int timeLed2 = 0;
long int timeLed3 = 0;
long int timeLed4 = 0;
long int timeLed5 = 0;
long int timeLed6 = 0;

float led1OutVal = 0.0;

long int currentTimeMS;
long int currentTimeMicr;

int arpTriggered = 0;
float arpNoteShift = 0.0;
bool forceArpTrigger = false;
float arpEnvVal;
long int arpSpeed = 0;
int arpNoteIter = 0;
long int arpTime = 0;
int arpAdvanceDirection = 1;
float arpInverterVal = 0.0;
float oldArpInverterVal = 0.0;
float arpEnvAttack = 0.0;
float arpNote = 0.0;
float arpKnobVal = 0.0;
float arpTrigger = 0.0;
float oldArpTrigger = 0.0;
bool arpDirectionGoingDown = false;
float arpInverterDelta = 0.0;
float arpTriggerDelta = 0.0;
bool arpTriggerGoingDown = false;
float arpNote2 = 0.0;
float inAudioFiltered = 0.0;
float inAudioToMidiFreq = 0.0;
float oldInAudioToMidiFreq = 0.0;
int inAudioToMidiIt = 0;
float inAudioToMidiAccum = 0.0;
bool inAudioToMidiActive = false;

long int randGenTime = 0;
float randomGenerator = 0.0;

float reverbWetLeft = 0.0;
float reverbWetRight = 0.0;

long int inAudio1 = 0.0;

float voice1 = 0.0;
float voice1BitcrushQuant = 0.0;
float voice1BitcrushDequant = 0.0;

float voice2 = 0.0;
float outEmitter = 1.0;

float getJackValueOut = 0.0;
int connectedOutJackId = 0;

long int knobsReadTime = 0;
int knobReadIter = 0;

long int randGenSpeed = 0;

float sampleRate = 0.0;
float halfSampleRate = 0.0;

bool waveFolderEnabled = false;
bool drumMachineEnabled = false;

Oscillator arpOsc;
AdEnv arpEnvelop;
ReverbSc reverb;
Oscillator lfo1;
Oscillator lfo2;
Fold voice1Fold;
Svf voice1Filter;
Oscillator voice2Osc;
Svf voice2Filter;
WhiteNoise voice2Noise;
SyntheticBassDrum drum;
Metro drumTick;

float lfo1OutVal = 0.0;
float lfo2OutVal = 0.0;
float arpOutVal = 0;

float filterLevel = 0.0;
float foldLevel = 0.0;
float foldAtten = 0.0;

float audioSig = 0.0;
float voice2Env = 0.0;
float voice2Freq = 0.0;
int voice2WaveForm = 0;

#define NUM_OUT_JACKS 5
#define NUM_IN_JACKS 8

int jackWriteIter = 0;
const int outJackPinId[NUM_OUT_JACKS] = {JACKOUT1, JACKOUT2, JACKOUT3, JACKOUT4, JACKOUT5};
int jackReadIter = 0;
const int inJackPinId[NUM_IN_JACKS] = {JACKIN1, JACKIN2, JACKIN3, JACKIN4, JACKIN5, JACKIN6, JACKIN7, JACKIN8};
int inJackConnectionsTmp[NUM_IN_JACKS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int inJackConnections[NUM_IN_JACKS] = {-1, -1, -1, -1, -1, -1, -1, -1};
int jackValRead = 0;

int printIter = 0;

// audio2midi code based on AudioFrequencyMeter https://github.com/arduino-libraries/AudioFrequencyMeter
#define AUDIO2MIDI_DEFAULT_TIMER_TOLERANCE         10
#define AUDIO2MIDI_DEFAULT_SLOPE_TOLERANCE         3
#define AUDIO2MIDI_ARRAY_DEPTH             20
#define AUDIO2MIDI_MIDPOINT                127
static int audio2midi_slopeTolerance;
static int audio2midi_timerTolerance;
static int  audio2midi_newData, audio2midi_prevData;
static unsigned int audio2midi_time, audio2midi_totalTimer; 
static volatile unsigned int audio2midi_period;
static volatile unsigned int audio2midi_oldPeriod;
static int audio2midi_arrayIndex; 
static int audio2midi_timer[AUDIO2MIDI_ARRAY_DEPTH];
static int audio2midi_slope[AUDIO2MIDI_ARRAY_DEPTH]; 
static int audio2midi_maxSlope;
static int audio2midi_newSlope;
static int audio2midi_noMatch;
static int audio2midi_maxAmplitude;   
static int audio2midi_newMaxAmplitude;
static volatile int audio2midi_checkMaxAmp;
float audio2midi_envelop;
//

inline float getJackValue(int jackId, float defaultVal){
	getJackValueOut = defaultVal;

	connectedOutJackId = inJackConnections[jackId - 1];
	if(connectedOutJackId != -1){
		if(connectedOutJackId == 0){
			getJackValueOut = lfo1OutVal;
		}
		else if(connectedOutJackId == 1){
			getJackValueOut = randomGenerator;
		}
		else if(connectedOutJackId == 2){
			if(inAudioToMidiActive){
				getJackValueOut = inAudioFiltered;
			}
			else{
				getJackValueOut = arpNote2;
			}
		}
		else if(connectedOutJackId == 3){
			getJackValueOut = lfo2OutVal;
		}
		else if(connectedOutJackId == 4){
			getJackValueOut = arpOutVal;
		}
		else if(connectedOutJackId == 5){
			getJackValueOut = knob5Val;
		}
	}

	return getJackValueOut;
}

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  float v = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if(v < out_min){
  	v = out_min;
  }
  else if(v > out_max){
  	v = out_max;
  }

  return v;
}

inline float mix(float a, float b, float x)
{ 
  return a + x * (b - a);
}

inline void audio2MidiUpdate(float inAudio){
	audio2midi_prevData = audio2midi_newData;
	audio2midi_newData = (int)round(((inAudio + 1.0) * 0.5) * 255);
	
	bool triggered = false; 

	if (audio2midi_prevData != audio2midi_newData && (audio2midi_prevData < AUDIO2MIDI_MIDPOINT) && (audio2midi_newData >= AUDIO2MIDI_MIDPOINT)) {
	  audio2midi_newSlope = audio2midi_newData - audio2midi_prevData;

	  if (abs(audio2midi_newSlope - audio2midi_maxSlope) < audio2midi_slopeTolerance) {
	    audio2midi_slope[audio2midi_arrayIndex] = audio2midi_newSlope;
	    audio2midi_timer[audio2midi_arrayIndex] = audio2midi_time;
	    audio2midi_time = 0;
	    
	    if (audio2midi_arrayIndex == 0) {
	      audio2midi_noMatch = 0;
	      audio2midi_arrayIndex++;
	    }
	    else if ((abs(audio2midi_timer[0] - audio2midi_timer[audio2midi_arrayIndex]) < audio2midi_timerTolerance) && (abs(audio2midi_slope[0] - audio2midi_newSlope) < audio2midi_slopeTolerance)) {
	      audio2midi_totalTimer = 0;
	      for (int i = 0; i < audio2midi_arrayIndex; i++) {
	        audio2midi_totalTimer += audio2midi_timer[i];
	      }
	      audio2midi_period = audio2midi_totalTimer;

	      audio2midi_envelop = mix(audio2midi_envelop, 1.0, 0.4);

	      audio2midi_timer[0] = audio2midi_timer[audio2midi_arrayIndex];
	      audio2midi_slope[0] = audio2midi_slope[audio2midi_arrayIndex];
	      audio2midi_arrayIndex = 1;
	      audio2midi_noMatch = 0;
	    }
	    else {
	      audio2midi_arrayIndex++;
	      if (audio2midi_arrayIndex > AUDIO2MIDI_ARRAY_DEPTH - 1) {
	        audio2midi_arrayIndex = 0;
	        audio2midi_noMatch = 0;
	        audio2midi_maxSlope = 0;
	      }
	    }
	  }
	  else if (audio2midi_newSlope > audio2midi_maxSlope) {
	    audio2midi_maxSlope = audio2midi_newSlope;
	    audio2midi_time = 0;
	    audio2midi_noMatch = 0;
	    audio2midi_arrayIndex = 0;
	  }
	  else {
	    audio2midi_noMatch++;
	    if (audio2midi_noMatch > AUDIO2MIDI_ARRAY_DEPTH - 1) {
	      audio2midi_arrayIndex = 0;
	      audio2midi_noMatch = 0;
	      audio2midi_maxSlope = 0;
	    }
	  }
	}
	
	audio2midi_time++;                             // Incremented at sampleRate

	audio2midi_newMaxAmplitude = abs(AUDIO2MIDI_MIDPOINT - audio2midi_newData);

	if (audio2midi_newMaxAmplitude > audio2midi_maxAmplitude) {
	  audio2midi_maxAmplitude = audio2midi_newMaxAmplitude;
	}
}

inline float audio2MidiGetFrequency(){
	return (float)(sampleRate / audio2midi_period);
}

inline void audio2MidiReset(){
	audio2midi_checkMaxAmp = audio2midi_maxAmplitude;
	audio2midi_maxAmplitude = 0;

	if(audio2midi_checkMaxAmp <= 1){
	  audio2midi_envelop *= 0.99;
	}

	audio2midi_oldPeriod = audio2midi_period;
}

void audioUpdate(float **in, float **out, size_t size) {
	inAudioToMidiIt += 1;
	inAudioToMidiAccum += abs(inAudioToMidiFreq - oldInAudioToMidiFreq);
	oldInAudioToMidiFreq = inAudioToMidiFreq;
	if(inAudioToMidiIt == 1000){
		if(inAudioToMidiAccum > 0.01){
			inAudioToMidiActive = true;
		}
		else{
			inAudioToMidiActive = false;
		}

		inAudioToMidiAccum = 0.0;
		inAudioToMidiIt = 0;
	}

	updateArpTrigger();

	voice2Freq = getJackValue(1, -1.0);
	voice2Env = getJackValue(2, -1.0);
	
	// if(voice2Freq != -1 && voice2Env == -1.0){
	// 	if(inAudioToMidiActive){
	// 		if(audio2midi_envelop > 0.0){
	// 			voice2Env = audio2midi_envelop;
	// 		}
	// 	}
	// 	else{
	// 		voice2Env = arpOutVal;
	// 	}
	// }

	voice2Osc.SetFreq(mtof(mix(ARPMIN2, ARPMAX2, voice2Freq)));

	lfo1OutVal = (lfo1.Process() + 1.0) * 0.5;
	lfo2OutVal = (lfo2.Process() + 1.0) * 0.5;

	if(waveFolderEnabled){
		foldAtten = mapFloat(foldLevel, 0.0, 1.0, 1.0, 0.5);
	}
	
	arpOutVal = 0.0;
	inAudio1 = 0;
	for (size_t i = 0; i < size; i++) {
		arpEnvVal = arpEnvelop.Process();

		arpOutVal += arpEnvVal;

		if(!drumMachineEnabled){
			voice1 = arpOsc.Process() * arpEnvVal * 0.5;
		}
		else{
			float drumTickVal = drumTick.Process();
			voice1 = drum.Process(drumTickVal);
		}

		if(!waveFolderEnabled){
			voice1 = voice1BitcrushDequant * (int)(voice1 * voice1BitcrushQuant);
		}
		else{
			voice1 = voice1Fold.Process(voice1 * foldAtten);
		}

		voice1Filter.Process(voice1);
		voice1 = voice1Filter.Low();

		if(voice2WaveForm < 8){
			voice2 = voice2Osc.Process() * voice2Env * 0.5;
		}
		else{
			voice2 = voice2Noise.Process() * voice2Env * 0.1;
		}

		voice2Filter.Process(voice2);
		voice2 = voice2Filter.Low();
		
		audioSig = (voice1 + voice2) * 0.5;

		reverb.Process(audioSig, audioSig, &reverbWetLeft, &reverbWetRight);

		out[0][i] = audioSig + (reverbWetLeft + reverbWetRight);
		
		inAudio1 += abs((in[1][i])) * 10000;
		
		out[1][i] = outEmitter;// this will keep out jackOut5 current reading alive
		outEmitter *= -1.0;

		audio2MidiUpdate(in[0][i]);
	}

	inAudioToMidiFreq = audio2MidiGetFrequency();
	inAudioFiltered = mix(inAudioFiltered, mapFloat(inAudioToMidiFreq, 100.0, 400.0, 0.0, 1.0), 0.01);
	
	audio2MidiReset();

	arpOutVal /= size;

	if(inAudio1 < 150){// this avoids getting spikes when rotating the knob
		knob1ValAccum += inAudio1;
		
		if(knob1ValAccumIt == 10){
			knob1Val = mix(knob1Val, mapFloat((float)knob1ValAccum * 0.005, 0.5, 1.0, 0.0, 1.0), 0.1);//mapFloat(knob1ValAccum, 400.0, 600.0, 0.0, 1.0);
			
			// inAudioFiltered = inAudio / 10;

			knob1ValAccum = 0;
			knob1ValAccumIt = 0;
		}

		knob1ValAccumIt += 1;
	}
	
	currentTimeMicr = micros();

	if(inAudioToMidiActive){
		led1OutVal = inAudioFiltered;
	}
	else{
		led1OutVal = arpNote2;
	}
	
	setLedFade(LED1, led1OutVal, currentTimeMicr, timeLed1);
	setLedFade(LED2, lfo1OutVal, currentTimeMicr, timeLed2);
	setLedFade(LED3, randomGenerator, currentTimeMicr, timeLed3);
	setLedFade(LED4, lfo2OutVal, currentTimeMicr, timeLed4);
	setLedFade(LED5, knob5Val, currentTimeMicr, timeLed5);
	setLedFade(LED6, arpOutVal, currentTimeMicr, timeLed6);
}

void resetToBootloader() {
  // Initialize Boot Pin
  dsy_gpio_pin bootpin = {DSY_GPIOG, 3};
  dsy_gpio pin;
  pin.mode = DSY_GPIO_MODE_OUTPUT_PP;
  pin.pin = bootpin;
  dsy_gpio_init(&pin);

  // Pull Pin HIGH
  dsy_gpio_write(&pin, 1);
    
  delay(10);
  HAL_NVIC_SystemReset();
}

bool bootBlink = false;

bool runInitSetup(){
	bool bootMode = false;

	pinMode(JACKOUT1, OUTPUT);
	pinMode(JACKOUT2, OUTPUT);
	pinMode(JACKOUT3, OUTPUT);
	pinMode(JACKOUT4, OUTPUT);
	pinMode(JACKOUT5, OUTPUT);

	pinMode(JACKIN1, OUTPUT);
	pinMode(JACKIN2, OUTPUT);
	pinMode(JACKIN3, OUTPUT);
	pinMode(JACKIN4, OUTPUT);
	pinMode(JACKIN5, OUTPUT);
	pinMode(JACKIN6, OUTPUT);
	pinMode(JACKIN7, OUTPUT);
	pinMode(JACKIN8, OUTPUT);

	digitalWrite(JACKOUT1, LOW);
	digitalWrite(JACKOUT2, LOW);
	digitalWrite(JACKOUT3, LOW);
	digitalWrite(JACKOUT4, LOW);
	digitalWrite(JACKOUT5, LOW);
	digitalWrite(JACKIN1, LOW);
	digitalWrite(JACKIN2, LOW);
	digitalWrite(JACKIN3, LOW);
	digitalWrite(JACKIN4, LOW);
	digitalWrite(JACKIN5, LOW);
	digitalWrite(JACKIN6, LOW);
	digitalWrite(JACKIN7, LOW);
	digitalWrite(JACKIN8, LOW);

	pinMode(JACKOUT1, OUTPUT);
	pinMode(JACKOUT2, INPUT);

	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	pinMode(LED5, OUTPUT);

	digitalWrite(JACKOUT1, HIGH);
	int value = digitalRead(JACKOUT2);
	if(value == HIGH){
		for(int i = 0; i < 10; ++i){
			if(bootBlink){
				digitalWrite(LED2, HIGH);
				digitalWrite(LED3, HIGH);
				digitalWrite(LED4, HIGH);
				digitalWrite(LED5, HIGH);
			}
			else{
				digitalWrite(LED2, LOW);
				digitalWrite(LED3, LOW);
				digitalWrite(LED4, LOW);
				digitalWrite(LED5, LOW);
			}

			bootBlink = !bootBlink;

			delay(100);
		}

		resetToBootloader();
	}
	else{
		value = digitalRead(JACKOUT3);

		if(value == HIGH){
			waveFolderEnabled = true;
		}
		else{
			value = digitalRead(JACKOUT4);
			if(value == HIGH){
				drumMachineEnabled = true;
			}
		}
	}

	return bootMode;
}

void audio2MidiInit(){
	audio2midi_slopeTolerance = AUDIO2MIDI_DEFAULT_SLOPE_TOLERANCE;
	audio2midi_timerTolerance = AUDIO2MIDI_DEFAULT_TIMER_TOLERANCE;
	
	audio2midi_newData = 0;
	audio2midi_prevData = AUDIO2MIDI_MIDPOINT;
	audio2midi_time = 0;
	audio2midi_arrayIndex = 0;
	audio2midi_maxSlope = 0;
	audio2midi_noMatch = 0;
	audio2midi_maxAmplitude = 0;
	audio2midi_checkMaxAmp = 0;
}

#define DATA_LENGTH E2END

void setup() {
	bool bootMode = runInitSetup();
	if(bootMode){
		
		return;
	}

	// leds
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	pinMode(LED5, OUTPUT);
	pinMode(LED6, OUTPUT);

	// knobs
	pinMode(KNOB2, INPUT);
	pinMode(KNOB3, INPUT);
	pinMode(KNOB4, INPUT);
	pinMode(KNOB5, INPUT);
	pinMode(KNOB6, INPUT);
	pinMode(KNOB7, INPUT);
	pinMode(KNOB8, INPUT);
	pinMode(KNOB9, INPUT);
	pinMode(KNOB10, INPUT);
	pinMode(KNOB11, INPUT);
	pinMode(KNOB12, INPUT);
	pinMode(KNOB13, INPUT);

	// in out jacks
	pinMode(JACKOUT1, OUTPUT);
	pinMode(JACKOUT2, OUTPUT);
	pinMode(JACKOUT3, OUTPUT);
	pinMode(JACKOUT4, OUTPUT);
	pinMode(JACKOUT5, OUTPUT);

	pinMode(JACKIN1, INPUT_PULLUP);
	pinMode(JACKIN2, INPUT_PULLUP);
	pinMode(JACKIN3, INPUT_PULLUP);
	pinMode(JACKIN4, INPUT_PULLUP);
	pinMode(JACKIN5, INPUT_PULLUP);
	pinMode(JACKIN6, INPUT_PULLUP);
	pinMode(JACKIN7, INPUT_PULLUP);
	pinMode(JACKIN8, INPUT_PULLUP);

	// Initialize for Daisy at 48kHz
	hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
	sampleRate = DAISY.get_samplerate();

	halfSampleRate = sampleRate * 0.5;

	arpOsc.Init(sampleRate);
	arpOsc.SetFreq(440);
	arpOsc.SetAmp(1.0);

	lfo1.Init(sampleRate);
	lfo1.SetFreq(440);
	lfo1.SetAmp(1.0);
	lfo1.SetWaveform(arpOsc.WAVE_SIN);

	lfo2.Init(sampleRate);
	lfo2.SetFreq(440);
	lfo2.SetAmp(1.0);
	lfo2.SetWaveform(arpOsc.WAVE_RAMP);

	arpEnvelop.Init(sampleRate);
	arpEnvelop.SetMax(1.f);
	arpEnvelop.SetMin(0.f);

	voice1Filter.Init(sampleRate);
	voice1Filter.SetRes(0.9);
	voice1Filter.SetDrive(0.3);

	reverb.Init(sampleRate);
	reverb.SetLpFreq(500.0f);

	if(waveFolderEnabled){
		voice1Fold.Init();
	}

	if(drumMachineEnabled){
		drum.Init(sampleRate);
		drumTick.Init(2.f, sampleRate);
	}

	voice2Osc.Init(sampleRate);

	voice2Filter.Init(sampleRate);
	voice2Filter.SetRes(0.9);
	voice2Filter.SetDrive(0.3);

	voice2Noise.Init();

	audio2MidiInit();

	DAISY.begin(audioUpdate);
}

inline void writeOutputJacks(){
	if(jackReadIter > NUM_OUT_JACKS){
		return;
	}

	for(int i = 0; i < NUM_OUT_JACKS; ++i){
		if(jackWriteIter <= i){
			digitalWrite(outJackPinId[i], HIGH);
		}
		else{
			digitalWrite(outJackPinId[i], LOW);
		}
	}

	jackWriteIter += 1;

	if(jackWriteIter == NUM_OUT_JACKS + 1){
		jackWriteIter = 0;
	}
}

inline void readInputJacks(){
	for(int i = 0; i < NUM_IN_JACKS; ++i){
		jackValRead = digitalRead(inJackPinId[i]);
		
		if(jackValRead == HIGH){
			inJackConnectionsTmp[i] += 1;
		}
	}

	jackReadIter += 1;

	if(jackReadIter == NUM_IN_JACKS + 14){// need to accumulate more values to be able to identify jack5
		jackReadIter = 0;

		for(int i = 0; i < NUM_IN_JACKS; ++i){
			jackValRead = inJackConnectionsTmp[i];

			if(jackValRead >= 21){
				jackValRead = -1;
			}
			else if(jackValRead > 4){
				jackValRead = 5;
			}

			inJackConnections[i] = jackValRead;
			
			inJackConnectionsTmp[i] = -1;
		}
	}
}

inline void setLedFade(uint pin, float &value, long int &currentTimeMicr, long int &outTime){
	if(currentTimeMicr - outTime > (int)mapFloat(1.0 - value, 0.0, 1.0, 2346, 23695)){
		digitalWrite(pin, HIGH);

		outTime = currentTimeMicr;
	}
	else{
		digitalWrite(pin, LOW);
	}
}

inline void arpeggiator(){
	// arp inverter
	arpInverterVal = getJackValue(8, -999.0);
	if(arpInverterVal != -999.0){
		arpInverterDelta = arpInverterVal - oldArpInverterVal;
		if(arpInverterDelta < -0.01 && arpDirectionGoingDown == false){
			arpAdvanceDirection = arpAdvanceDirection * -1;

			arpDirectionGoingDown = true;
		}
		else{
			if(arpInverterDelta > 0.01){
				arpDirectionGoingDown = false;
			}
		}

		oldArpInverterVal = arpInverterVal;
	}
	else{
		arpAdvanceDirection = 1;
	}
	/////

	// arp sequencer
	arpNoteShift = 0.0;
	// if(arpTrigger != -999.0){
	// 	arpNoteShift = constrain(mapFloat(knob6Val, 0.0, 1.0, -0.5, 0.5), 0.0, 1.0);
	// }

	arpEnvelop.SetTime(ADENV_SEG_DECAY, knob7Val);

	arpEnvAttack = knob8Val;
	if(arpEnvAttack < 0.001){
		arpEnvAttack = 0.001;
	}
	arpEnvelop.SetTime(ADENV_SEG_ATTACK, arpEnvAttack);

	arpNote = 0.0;
	arpKnobVal = 0.0;
	if(arpNoteIter == 0){
		arpKnobVal = knob10Val;
		arpNote = mtof(((knob10Val + arpNoteShift) * ARPRANGE) + ARPMIN);

		arpNote2 = knob12Val;
	}
	else if(arpNoteIter == 1){
		arpKnobVal = knob11Val;
		arpNote = mtof(((knob11Val + arpNoteShift) * ARPRANGE) + ARPMIN);

		arpNote2 = knob13Val;
	}
	else if(arpNoteIter == 2){
		arpKnobVal = knob12Val;
		arpNote = mtof(((knob12Val + arpNoteShift) * ARPRANGE) + ARPMIN);

		arpNote2 = knob10Val;
	}
	else if(arpNoteIter == 3){
		arpKnobVal = knob13Val;
		arpNote = mtof(((knob13Val + arpNoteShift) * ARPRANGE) + ARPMIN);

		arpNote2 = knob11Val;
	}

	if(arpKnobVal > 0.01){
		arpOsc.SetFreq(arpNote);
		arpEnvelop.Trigger();

		if(drumMachineEnabled){
			drum.SetFreq(arpNote);
		}
	}

	arpNoteIter += arpAdvanceDirection;
	if(arpNoteIter == 4){
		arpNoteIter = 0;
	}
	else if(arpNoteIter == -1){
		arpNoteIter = 3;
	}
}

inline void updateControls(long int &currentTimeMS){
	if(knobReadIter == 0){
		knob2Val = (float)analogRead(KNOB2) * 0.0009775171;
	}
	else if(knobReadIter == 1){
		knob3Val = (float)analogRead(KNOB3) * 0.0009775171;
	}
	else if(knobReadIter == 2){
		knob4Val = (float)analogRead(KNOB4) * 0.0009775171;
	}
	else if(knobReadIter == 3){
		knob5Val = (float)analogRead(KNOB5) * 0.0009775171;
	}
	else if(knobReadIter == 4){
		knob6Val = (float)analogRead(KNOB6) * 0.0009775171;
	}
	else if(knobReadIter == 5){
		knob7Val = (float)analogRead(KNOB7) * 0.0009775171;
	}
	else if(knobReadIter == 6){
		knob8Val = (float)analogRead(KNOB8) * 0.0009775171;
	}
	else if(knobReadIter == 7){
		knob9Val = (float)analogRead(KNOB9) * 0.0009775171;
	}
	else if(knobReadIter == 8){
		knob10Val = (float)analogRead(KNOB10) * 0.0009775171;
	}
	else if(knobReadIter == 9){
		knob11Val = (float)analogRead(KNOB11) * 0.0009775171;
	}
	else if(knobReadIter == 10){
		knob12Val = (float)analogRead(KNOB12) * 0.0009775171;
	}
	else if(knobReadIter == 11){
		knob13Val = (float)analogRead(KNOB13) * 0.0009775171;
	}
	
	lfo1.SetFreq((knob2Val * 200) + 0.1);
	lfo2.SetFreq((knob4Val * 200) + 0.1);

	reverb.SetFeedback(knob1Val * 0.7);
	reverb.SetLpFreq(knob1Val * (halfSampleRate * 0.5));

	filterLevel = getJackValue(4, 1.0);

	if(waveFolderEnabled){
		foldLevel = getJackValue(5, 0.0);
		voice1Fold.SetIncrement(foldLevel * 100.0);
	}

	voice1Filter.SetFreq(mapFloat(filterLevel, 0.0, 1.0, halfSampleRate * 0.01, halfSampleRate));

	voice1BitcrushQuant = pow(2, mapFloat(getJackValue(5, 1.0), 0.0, 1.0, 1.0, 16.0));
	if(voice1BitcrushQuant <= 0.0){
		voice1BitcrushQuant = 0.00001;
	}

	voice1BitcrushDequant = 1.0f / voice1BitcrushQuant;

	if(drumMachineEnabled){
		drum.SetTone(0.5);
		drum.SetDecay(knob7Val);
		drum.SetFmEnvelopeAmount(knob8Val);
		drum.SetFmEnvelopeDecay(knob9Val);
		drumTick.SetFreq(knob6Val * 10.0);
	}

	voice2Filter.SetFreq(mapFloat(getJackValue(6, 1.0), 0.0, 1.0, halfSampleRate * 0.01, halfSampleRate));

	arpOsc.SetWaveform((int)(knob9Val * 7.0));

	voice2WaveForm = (int)(getJackValue(7, 0) * 10.0);

	if(voice2WaveForm < 8){
		voice2Osc.SetWaveform((int)(voice2WaveForm));
	}

	knobsReadTime = currentTimeMS;

	knobReadIter += 1;
	if(knobReadIter == 12){
		knobReadIter = 0;
	}
}

inline void updateArpTrigger(){
	arpTrigger = getJackValue(3, -999.0);
	if(arpTrigger < -1.0){
		arpSpeed = ((1.0 - knob6Val) * 2000) + 5;
	}
	else{
		arpSpeed = 99999;// never trigger by time

		arpTriggerDelta = arpTrigger - oldArpTrigger;
		if(arpTriggerDelta < -0.001 && arpTriggerGoingDown == false){
			arpTriggerGoingDown = true;

			forceArpTrigger = true;
		}

		if(arpTriggerDelta > 0.001){
			arpTriggerGoingDown = false;
		}
		
		oldArpTrigger = arpTrigger;
	}
}

void loop(){
	writeOutputJacks();
	readInputJacks();

	currentTimeMS = millis();

	if(currentTimeMS - knobsReadTime > 2){
		updateControls(currentTimeMS);
	}

	arpTriggered = 0;
	if(currentTimeMS - arpTime > arpSpeed && arpSpeed < 99999){
		arpTriggered = 1;
	}

	if(forceArpTrigger){
		arpTriggered = 1;
	}

	if(arpTriggered == 1){
		arpeggiator();

		forceArpTrigger = false;

		arpTime = currentTimeMS;
	}

	randGenSpeed = ((1.0 - knob3Val) * 2000) + 5;
	if(currentTimeMS - randGenTime > randGenSpeed){
		randomGenerator = (float)random(0, 100) / 100;

		randGenTime = currentTimeMS;
	}

	// printIter += 1;
	// if(printIter == 1000){
	// 	Serial.println(5);
		
	// 	printIter = 0;
	// }
}

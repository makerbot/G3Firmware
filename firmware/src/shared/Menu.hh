#ifndef MENU_HH_
#define MENU_HH_

#include "Types.hh"
#include "ButtonArray.hh"
#include "LiquidCrystal.hh"

/// The screen class defines a standard interface for anything that should
/// be displayed on the LCD.
class Screen {
public:
        /// Get the rate that this display should be updated. This is called
        /// after every screen display, so it can be used to dynamically
        /// adjust the update rate. This can be as fast as you like, however
        /// refreshing too fast during a build is certain to interfere with
        /// the serial and stepper processes, which will decrease build quality.
        /// \return refresh interval, in microseconds.
	virtual micros_t getUpdateRate();

        /// Update the screen display,
        /// \param[in] lcd LCD to write to
        /// \param[in] forceRedraw if true, redraw the entire screen. If false,
        ///                        only updated sections need to be redrawn.
	virtual void update(LiquidCrystal& lcd, bool forceRedraw);

        /// Reset the screen to it's default state
	virtual void reset();

        /// Get a notification that a button was pressed down.
        /// This function is called for every button that is pressed. Screen
        /// logic can be updated, however the screen should not be redrawn
        /// until update() is called again.
        ///
        /// Note that the current implementation only supports one button
        /// press at a time, and will discard any other events.
        /// \param button Button that was pressed
        virtual void notifyButtonPressed(ButtonArray::ButtonName button);
};


/// The menu object can be used to display a list of options on the LCD
/// screen. It handles updating the display and responding to button presses
/// automatically.
class Menu: public Screen {
public:
	virtual micros_t getUpdateRate() {return 500L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

	virtual void resetState();

        void notifyButtonPressed(ButtonArray::ButtonName button);

protected:

        uint8_t itemIndex;              ///< The currently selected item
        uint8_t lastDrawIndex;          ///< The index used to make the last draw
        uint8_t itemCount;              ///< Total number of items
        uint8_t firstItemIndex;         ///< The first selectable item. Set this
                                        ///< to greater than 0 if the first
                                        ///< item(s) are a title)

        /// Draw an item at the current cursor position.
        /// \param[in] index Index of the item to draw
        /// \param[in] LCD screen to draw onto
	virtual void drawItem(uint8_t index, LiquidCrystal& lcd);

        /// Handle selection of a menu item
        /// \param[in] index Index of the menu item that was selected
	virtual void handleSelect(uint8_t index);

        /// Handle the menu being cancelled. This should either remove the
        /// menu from the stack, or pop up a confirmation dialog to make sure
        /// that the menu should be removed.
	virtual void handleCancel();
};

/// Display a welcome splash screen, that removes itself when updated.
class SplashScreen: public Screen {
public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class UserViewMenu: public Menu {
public:
	UserViewMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class JogMode: public Screen {
private:
	enum distance_t {
	  DISTANCE_0_1MM = 0,
	  DISTANCE_1MM,
	  DISTANCE_CONT,
	};

	UserViewMenu userViewMenu;

	distance_t jogDistance;
	bool distanceChanged;
	bool userViewMode;
	bool userViewModeChanged;
	ButtonArray::ButtonName lastDirectionButtonPressed;

        void jog(ButtonArray::ButtonName direction);

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

/// This is an easter egg.
class SnakeMode: public Screen {

#define MAX_SNAKE_SIZE 20      ///< Maximum length our snake can grow to
#define APPLES_BEFORE_GROW 4   ///< Number of apples the snake must eat before growing
#define START_SPEED  60        ///< Starting speed, in screen refresh times per turn


private:
	micros_t updateRate;

	struct coord_t {
		int8_t x;
		int8_t y;
	};

	enum direction_t {
	  DIR_NORTH,
	  DIR_EAST,
	  DIR_SOUTH,
	  DIR_WEST
	};

	int snakeLength;					// Length of our snake; this grows for every x 'apples' eaten
	coord_t snakeBody[MAX_SNAKE_SIZE];	// Table of each piece of the snakes body
	bool snakeAlive;					// The state of our snake
	direction_t snakeDirection;			// The direction the snake is heading
	coord_t applePosition;				// Location of the apple
	uint8_t applesEaten;				// Number of apples that have been eaten
//	int gameSpeed = START_SPEED;		// Speed of the game (in ms per turn)


public:
	micros_t getUpdateRate() {return updateRate;}

	// Refresh the display information
	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

	// Get notified that a button was pressed
        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class UnableToOpenFileMenu: public Menu {
public:
	UnableToOpenFileMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class SDMenu: public Menu {
private:
	uint8_t updatePhase;
	uint8_t lastItemIndex;
	bool	drawItemLockout;
	UnableToOpenFileMenu unableToOpenFileMenu;
public:
	SDMenu();

	void resetState();

	micros_t getUpdateRate() {return 500L * 1000L;}
	void notifyButtonPressed(ButtonArray::ButtonName button);

	void update(LiquidCrystal& lcd, bool forceRedraw);
protected:
	uint8_t countFiles();

        bool getFilename(uint8_t index,
                         char buffer[],
                         uint8_t buffer_size);

	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};


class PauseMode: public Screen {
private:
	ButtonArray::ButtonName lastDirectionButtonPressed;

        void jog(ButtonArray::ButtonName direction);

	uint8_t pauseState;

public:
	bool autoPause;

	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class PauseAtZPosScreen: public Screen {
private:
	float pauseAtZPos;

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class CancelBuildMenu: public Menu {
public:
	CancelBuildMenu();

	void resetState();

protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
private:
	PauseMode		pauseMode;
	bool			pauseDisabled;
	PauseAtZPosScreen	pauseAtZPosScreen;
	bool			printAnotherEnabled;
};

class MonitorMode: public Screen {
private:
	CancelBuildMenu cancelBuildMenu;

	enum UpdatePhase {
		UPDATE_PHASE_FIRST = 0,
		UPDATE_PHASE_TOOL_TEMP = UPDATE_PHASE_FIRST,
		UPDATE_PHASE_TOOL_TEMP_SET_POINT,
		UPDATE_PHASE_PLATFORM_TEMP,
		UPDATE_PHASE_PLATFORM_SET_POINT,
		UPDATE_PHASE_BUILD_PHASE_SCROLLER,
		UPDATE_PHASE_LAST	//Not counted, just an end marker
	};

	enum BuildTimePhase {
		BUILD_TIME_PHASE_FIRST = 0,
		BUILD_TIME_PHASE_COMPLETED_PERCENT = BUILD_TIME_PHASE_FIRST,
		BUILD_TIME_PHASE_ELAPSED_TIME,
		BUILD_TIME_PHASE_TIME_LEFT,
		BUILD_TIME_PHASE_ZPOS,
		BUILD_TIME_PHASE_FILAMENT,
		BUILD_TIME_PHASE_COPIES_PRINTED,
		BUILD_TIME_PHASE_LAST	//Not counted, just an end marker
	};

	enum UpdatePhase updatePhase;
	enum BuildTimePhase buildTimePhase, lastBuildTimePhase;
	float   lastElapsedSeconds;
	PauseMode pauseMode;
	bool	pausePushLockout;
	bool buildCompleteBuzzPlayed;
	int32_t buildDuration;
	bool	overrideForceRedraw;
	uint8_t	copiesPrinted;
	bool	timeLeftDisplayed;

public:
	micros_t getUpdateRate() {return 500L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class VersionMode: public Screen {
private:

public:
	micros_t getUpdateRate() {return 500L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class Tool0TempSetScreen: public Screen {
private:
	uint8_t value;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class PlatformTempSetScreen: public Screen {
private:
	uint8_t value;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class PreheatMenu: public Menu {
public:
	PreheatMenu();

	void fetchTargetTemps();

protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);

private:
	uint16_t tool0Temp;
	uint16_t platformTemp;

        /// Static instances of our menus
        Tool0TempSetScreen tool0TempSetScreen;
        PlatformTempSetScreen platTempSetScreen;
};

class ExtruderTooColdMenu: public Menu {
public:
	ExtruderTooColdMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class ExtruderSetRpmScreen: public Screen {
private:
	uint8_t rpm;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class ExtruderMode: public Screen {
private:
	enum extrudeSeconds {
		EXTRUDE_SECS_CANCEL = 0,
		EXTRUDE_SECS_1S     = 1,
		EXTRUDE_SECS_2S     = 2,
		EXTRUDE_SECS_5S     = 5,
		EXTRUDE_SECS_10S    = 10,
		EXTRUDE_SECS_30S    = 30,
		EXTRUDE_SECS_60S    = 60,
		EXTRUDE_SECS_90S    = 90,
		EXTRUDE_SECS_120S   = 120,
		EXTRUDE_SECS_240S   = 240,
	};

	enum extrudeSeconds extrudeSeconds;
	bool timeChanged;
	int16_t lastDirection;
	ExtruderTooColdMenu extruderTooColdMenu;
        ExtruderSetRpmScreen extruderSetRpmScreen;

	uint8_t updatePhase;

	void extrude(seconds_t steps, bool overrideTempCheck);

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

	void notifyButtonPressed(ButtonArray::ButtonName button);
};

class MoodLightSetRGBScreen: public Screen {
private:
	uint8_t red;
	uint8_t green;
	uint8_t blue;

	int inputMode;	//0 = red, 1 = green, 2 = blue
	bool redrawScreen;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};


class MoodLightMode: public Screen {
private:
	uint8_t updatePhase;

	uint8_t scriptId;

        MoodLightSetRGBScreen   moodLightSetRGBScreen;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

	void notifyButtonPressed(ButtonArray::ButtonName button);
};

class HomeAxisMode: public Screen {
private:
        void home(ButtonArray::ButtonName direction);

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class SteppersMenu: public Menu {
public:
	SteppersMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class TestEndStopsMode: public Screen {
private:

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class AdvanceABPMode: public Screen {
private:
	bool abpForwarding;

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class CalibrateMode: public Screen {
private:
	enum calibrateState {
		CS_NONE,
		CS_START1,	//Disable steppers
		CS_START2,	//Disable steppers
		CS_PROMPT_MOVE,	//Prompt user to move build platform
		CS_HOME_Z,
		CS_HOME_Z_WAIT,
		CS_HOME_Y,
		CS_HOME_Y_WAIT,
		CS_HOME_X,
		CS_HOME_X_WAIT,
		CS_PROMPT_CALIBRATED
	};

	enum calibrateState calibrationState, lastCalibrationState;

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class HomeOffsetsMode: public Screen {
private:
	enum homeOffState {
		HOS_NONE,
		HOS_OFFSET_X,
		HOS_OFFSET_Y,
		HOS_OFFSET_Z,
	};

	enum homeOffState homeOffsetState, lastHomeOffsetState;

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class BuzzerSetRepeatsMode: public Screen {
private:
	uint8_t repeats;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class ExtruderFanMenu: public Menu {
public:
	ExtruderFanMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class StepsPerMMMode: public Screen {
private:
	enum StepsPerMMState {
		SPM_NONE,
		SPM_SET_X,
		SPM_SET_Y,
		SPM_SET_Z,
		SPM_SET_A
	};

	enum StepsPerMMState stepsPerMMState, lastStepsPerMMState;

	uint8_t	cursorLocation;

	int64_t originalStepsPerMM;

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class FilamentUsedResetMenu: public Menu {
public:
	FilamentUsedResetMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class FilamentUsedMode: public Screen {
private:
	FilamentUsedResetMenu filamentUsedResetMenu;

	bool overrideForceRedraw;
	bool lifetimeDisplay;
public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class ABPCopiesSetScreen: public Screen {
private:
	uint8_t value;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class PreheatDuringEstimateMenu: public Menu {
public:
	PreheatDuringEstimateMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class OverrideGCodeTempMenu: public Menu {
public:
	OverrideGCodeTempMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class StepperDriverAcceleratedMenu: public Menu {
public:
	StepperDriverAcceleratedMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class AcceleratedSettingsMode: public Screen {
private:
	enum accelerateSettingsState {
		AS_NONE,
		AS_MAX_FEEDRATE_X,
		AS_MAX_FEEDRATE_Y,
		AS_MAX_FEEDRATE_Z,
		AS_MAX_FEEDRATE_A,
		AS_MAX_ACCELERATION_X,
		AS_MAX_ACCELERATION_Y,
		AS_MAX_ACCELERATION_Z,
		AS_MAX_ACCELERATION_A,
		AS_MAX_EXTRUDER_NORM,
		AS_MAX_EXTRUDER_RETRACT,
		AS_MIN_FEED_RATE,
		AS_MIN_TRAVEL_FEED_RATE,
		AS_MAX_XY_JERK,
		AS_MAX_Z_JERK,
		AS_ADVANCE_K,
		AS_FILAMENT_DIAMETER,
	};

	enum accelerateSettingsState accelerateSettingsState, lastAccelerateSettingsState;

	uint32_t values[16];

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class EStepsPerMMLengthMode: public Screen {
private:
	uint32_t value;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);

	int32_t steps;
};

class EStepsPerMMStepsMode: public Screen {
private:
	int32_t value;
	ExtruderTooColdMenu extruderTooColdMenu;
	EStepsPerMMLengthMode eStepsPerMMLengthMode;

	void extrude(bool overrideTempCheck);

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class EStepsPerMMMode: public Screen {
private:
	uint32_t value;
	EStepsPerMMStepsMode eStepsPerMMStepsMode;

public:
	micros_t getUpdateRate() {return 100L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class AccelerationMenu: public Menu {
private:
	StepperDriverAcceleratedMenu	stepperDriverAcceleratedMenu;
	AcceleratedSettingsMode		acceleratedSettingsMode;
	EStepsPerMMMode			eStepsPerMMMode;
	
	bool acceleration;
public:
	AccelerationMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class BuildSettingsMenu: public Menu {
private:
	PreheatDuringEstimateMenu	preheatDuringEstimateMenu;
	OverrideGCodeTempMenu		overrideGCodeTempMenu;
	ABPCopiesSetScreen		abpCopiesSetScreen;
	AccelerationMenu		accelerationMenu;
public:
	BuildSettingsMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class ProfileChangeNameMode: public Screen {
private:
	uint8_t	cursorLocation;
	uint8_t profileName[8+1];

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);

	uint8_t profileIndex;
};

class ProfileDisplaySettingsMenu: public Menu {
private:
	uint8_t profileName[8+1];
	int32_t homeX, homeY, homeZ;
	uint8_t hbpTemp, tool0Temp, tool1Temp, extruderRpm;
public:
	ProfileDisplaySettingsMenu();

	void resetState();

	uint8_t profileIndex;
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class ProfileSubMenu: public Menu {
private:
	ProfileChangeNameMode	   profileChangeNameMode;
	ProfileDisplaySettingsMenu profileDisplaySettingsMenu;

public:
	ProfileSubMenu();

	void resetState();

	uint8_t profileIndex;
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class ProfilesMenu: public Menu {
private:
	ProfileSubMenu profileSubMenu;
public:
	ProfilesMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};

class CurrentPositionMode: public Screen {
private:

public:
	micros_t getUpdateRate() {return 50L * 1000L;}

	void update(LiquidCrystal& lcd, bool forceRedraw);

	void reset();

        void notifyButtonPressed(ButtonArray::ButtonName button);
};

class MainMenu: public Menu {
public:
	MainMenu();

protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);

private:
        /// Static instances of our menus
        MonitorMode monitorMode;
        SDMenu sdMenu;
        JogMode jogger;
	PreheatMenu preheatMenu;
	ExtruderMode extruderMenu;
	HomeAxisMode homeAxisMode;
	SteppersMenu steppersMenu;
	AdvanceABPMode advanceABPMode;
	BuzzerSetRepeatsMode buzzerSetRepeats;
	BuildSettingsMenu buildSettingsMenu;
	ProfilesMenu profilesMenu;
	ExtruderFanMenu extruderFanMenu;
	CalibrateMode calibrateMode;
	HomeOffsetsMode homeOffsetsMode;
	StepsPerMMMode stepsPerMMMode;
	FilamentUsedMode filamentUsedMode;
	CurrentPositionMode currentPositionMode;
	TestEndStopsMode testEndStopsMode;
        VersionMode versionMode;
	MoodLightMode	moodLightMode;
        SnakeMode snake;

	int64_t checkAndGetEepromDefault(const uint16_t location, const int64_t default_value);
};

#endif

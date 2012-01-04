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


class JogMode: public Screen {
private:
	enum distance_t {
	  DISTANCE_SHORT,
	  DISTANCE_LONG,
	  DISTANCE_CONT,
	};

	distance_t jogDistance;
	bool distanceChanged;
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


class SDMenu: public Menu {
private:
	uint8_t updatePhase;
	uint8_t lastItemIndex;
	bool	drawItemLockout;
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


class CancelBuildMenu: public Menu {
public:
	CancelBuildMenu();

	void resetState();
protected:
	void drawItem(uint8_t index, LiquidCrystal& lcd);

	void handleSelect(uint8_t index);
};


class MonitorMode: public Screen {
private:
	CancelBuildMenu cancelBuildMenu;

	uint8_t updatePhase;
	uint8_t buildTimePhase;
	float   lastElapsedSeconds;
	float   extruderStartSeconds; 
	bool	buildComplete;		//For solving floating point rounding issues

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
	TestEndStopsMode testEndStopsMode;
        VersionMode versionMode;
	MoodLightMode	moodLightMode;
        SnakeMode snake;
};

#endif

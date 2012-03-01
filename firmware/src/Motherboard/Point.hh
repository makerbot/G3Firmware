#ifndef POINT_HH
#define POINT_HH

#include "Configuration.hh"
#include <stdint.h>


#define AXIS_COUNT STEPPER_COUNT

/// Class that represents an N-dimensional point, where N is the number of
/// stepper axes present in the system. Can support 3 or 5 axes.
class Point {
private:
        int32_t coordinates[AXIS_COUNT];        ///< n-dimensional coordinate
public:
        /// Default point constructor
        Point() {}

        /// Construct a point with the given cooridnates. Coordinates are in
        /// stepper steps.
        /// \param[in] x X axis position
        /// \param[in] y Y axis position
        /// \param[in] z Z axis position
        /// \param[in] a (if supported) A axis position
        /// \param[in] b (if supported) B axis position
#if AXIS_COUNT > 3
        Point(int32_t x, int32_t y, int32_t z, int32_t a = 0, int32_t b = 0)
#else
        Point(int32_t x, int32_t y, int32_t z)
#endif
        {
            coordinates[0] = x;
            coordinates[1] = y;
            coordinates[2] = z;
#if AXIS_COUNT > 3
            coordinates[3] = a;
            coordinates[4] = b;
#endif
        }


        /// Constant array accessor.
        /// \param[in] index Axis to look up
        /// \return Axis position, in steps
        inline const int32_t& operator[](unsigned int index) const { return coordinates[index]; }


        /// Array accessor.
        /// \param[in] index Axis to look up
        /// \return Reference to the variable containing the axis' position.
        inline int32_t& operator[](unsigned int index) { return coordinates[index];}
        
} __attribute__ ((__packed__));


#endif // POINT_HH

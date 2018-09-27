#ifndef POSITION_H
#define POSITION_H

class Position{
private:
	double xMeters; // x-position in meters
	double yMeters; // y-position in meters
	double orientationRad; // radian in range of 0 to 2PI

public:
	double x() const;

	void x(double value);

	double y() const;

	void y(double value);

	double orientation() const;

	void orientation(double value);

	//constructor
	Position();

	Position(double x, double y);

	Position(double x, double y, double orientation);

	//calculates direct distance according to sqrt(dx^2+dy^2)
    double getDistanceTo(const Position &b) const;


};

Position::Position(){}

#endif // POSITION_H

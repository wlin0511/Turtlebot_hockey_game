
// Abstand zwischen zwei 2D Punkten (Polar)
double Localisation::distancePolar(const double &depthA, const double &depthB, const double &angleBetweenAB){
    // c² = a² + b² - 2*ab*cos(gamma)
    return qSqrt(depthA * depthA +
                 depthB * depthB -
                 2 * depthA * depthB *
                 qCos(angleBetweenAB));




Position Position::getGlobalPosition(const Position &poleA, const Position &poleB, const Position &poleC)
{

    QPair<double,double> outerIntersection;

    double delta_X;
    double delta_Y;
    double orientation;
    double worldX;
    double worldY;

    int retVal= trilateration::circle_circle_intersection(0, config::geoPol_1_2, poleB.x(),
                                                          0, config::geoPol_1_2 + config::geoPol_2_3, poleC.x(),
                                                          &outerIntersection.first, &outerIntersection.second,
                                                          &delta_X, &delta_Y);

    if(!retVal){
        if(config::enableDebugOrientation)
            qWarning()<<"Error during cycle intersection";
        return Position();

    }

    //    orientation = atan2(yiprime,xiprime)+poleA.second-M_PI_2;
    if(poleA.y() <= poleC.y())
    {
        //links
        orientation = M_PI_2 + atan2(delta_Y,delta_X)+poleA.y();
        worldY = delta_Y;
        worldX = delta_X;
    }
    else
    {
        //rechts
        //Spiegeln der x Koordinaten an der rechten Aussenlinie
        orientation = -atan2(delta_Y,delta_X)+poleA.y() - M_PI_2;
        worldX = config::geoPol_1_14 - delta_X;
        worldY = delta_Y;

    }
    return Position(worldX,worldY,orientation);
}


Position Localisation::getGlobalPolarPosition(const Position &relativePosition, const Position &robotPosition){
    double absAngle = fmod(robotPosition.orientation() - (relativePosition.y() - M_PI_2), 2*M_PI);

    return Position(robotPosition.x() + config::ORIENTATION_SENSOR_ODOMETRIE_DELTA * cos(robotPosition.rot()) + relativePosition.x() * cos(absAngle),
                    robotPosition.y() + config::ORIENTATION_SENSOR_ODOMETRIE_DELTA * sin(robotPosition.rot()) + relativePosition.x() * sin(absAngle),
                    absAngle,
                    relativePosition.sizeType());
}


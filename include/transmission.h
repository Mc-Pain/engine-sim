#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "scs.h"
#include "utilities.h"

class Transmission {
    public:
        struct Parameters {
            int GearCount;
            const double *GearRatios;
            double MaxClutchTorque;
        };

    public:
        Transmission();
        ~Transmission();

        void initialize(const Parameters &params);
        void update(double dt);
        void addToSystem(
            atg_scs::RigidBodySystem *system,
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        inline int getGear() const { return m_gear; }
        inline void setClutchPressure(double pressure) { m_clutchPressure = pressure; }
        inline double getClutchPressure() const { return m_clutchPressure; }
        inline void setTargetClutchPressure(double pressure) { m_targetClutchPressure = clamp(pressure); }
        inline double getTargetClutchPressure() const { return m_targetClutchPressure; }
        void updateClutchPressure(double clutch_s);
        inline bool isShifting() const { return m_upshift || m_downshift; }

        void upshift();
        void downshift();
        void recalculateTargetRPM();

    protected:
        atg_scs::ClutchConstraint m_clutchConstraint;
        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;
        Engine *m_engine;

        int m_gear;
        int m_newGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure = 1.0;
        double m_targetClutchPressure = 1.0;
        double m_targetRPM;

        bool m_upshift;
        bool m_downshift;
        double m_storedThrottle = 0.0;

        void changeGear(int newGear);
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */

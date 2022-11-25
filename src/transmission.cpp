#include "../include/transmission.h"

#include "../include/units.h"

#include <cmath>

Transmission::Transmission() {
    m_gear = -1;
    m_newGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_clutchPressure = 0.0;
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }

    m_gearRatios = nullptr;
}

void Transmission::initialize(const Parameters &params) {
    m_gearCount = params.GearCount;
    m_maxClutchTorque = params.MaxClutchTorque;
    m_gearRatios = new double[params.GearCount];
    memcpy(m_gearRatios, params.GearRatios, sizeof(double) * m_gearCount);
}

void Transmission::update(double dt) {
    if (m_gear == -1) {
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
    }
    else {
        m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure;
        m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure;
    }
}

void Transmission::addToSystem(
    atg_scs::RigidBodySystem *system,
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    m_rotatingMass = rotatingMass;
    m_vehicle = vehicle;
    m_engine = engine;

    m_clutchConstraint.setBody1(&engine->getOutputCrankshaft()->m_body);
    m_clutchConstraint.setBody2(m_rotatingMass);

    system->addConstraint(&m_clutchConstraint);
}

void Transmission::upshift() {
    int oldGear = m_gear;

    changeGear(m_gear + 1);

    // can't shift further
    if (oldGear == m_gearCount - 1) {
        return;
    }

    if (!isShifting()) {
        m_storedThrottle = m_engine->getTargetSpeedControl();
    }
    m_upshift = true;
    m_targetClutchPressure = 0.0;
    m_engine->setTargetSpeedControl(0.0);
}

void Transmission::downshift() {
    changeGear(m_gear - 1);

    // neutral
    if (m_gear == -1) {
        return;
    }

    if (!isShifting()) {
        m_storedThrottle = m_engine->getTargetSpeedControl();
    }
    m_downshift = true;
    m_targetClutchPressure = 0.0;
    m_engine->setTargetSpeedControl(1.0);
}

void Transmission::changeGear(int newGear) {
    if (newGear < -1 || newGear >= m_gearCount) return;
    else if (newGear != -1) {
        const double m_car = m_vehicle->getMass();
        const double gear_ratio = m_gearRatios[newGear];
        const double diff_ratio = m_vehicle->getDiffRatio();
        const double tire_radius = m_vehicle->getTireRadius();
        const double f = tire_radius / (diff_ratio * gear_ratio);

        const double new_I = m_car * f * f;
        const double E_r =
            0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
        const double new_v_theta = m_rotatingMass->v_theta < 0
            ? -std::sqrt(E_r * 2 / new_I)
            : std::sqrt(E_r * 2 / new_I);

        m_rotatingMass->I = new_I;
        m_rotatingMass->p_x = m_rotatingMass->p_y = 0;
        m_rotatingMass->m = m_car;
        m_rotatingMass->v_theta = new_v_theta;
    }

    m_gear = newGear;
}

void Transmission::updateClutchPressure(double clutch_s) {
    m_clutchPressure = m_clutchPressure * (1 - clutch_s) + m_targetClutchPressure * clutch_s;

    double vs = units::convert(m_vehicle->getSpeed(), units::m / units::minute);
    double tr = units::convert(m_vehicle->getTireRadius(), units::m);
    double wheelRPM = (vs / tr) / (constants::pi * 2);
    double targetRPM = (m_gear != -1) ? (wheelRPM * m_vehicle->getDiffRatio() * m_gearRatios[m_gear]) : 0.0;

    if (m_upshift && m_engine->getRpm() < targetRPM) {
        m_upshift = false;
        m_engine->setTargetSpeedControl(m_storedThrottle);
        m_targetClutchPressure = 1.0;
    }

    if (m_downshift && m_engine->getRpm() > targetRPM) {
        m_downshift = false;
        m_engine->setTargetSpeedControl(m_storedThrottle);
        m_targetClutchPressure = 1.0;
    }
}

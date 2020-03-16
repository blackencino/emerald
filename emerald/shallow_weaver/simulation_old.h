
#include <emerald/shallow_weaver/simulation.h>
#include <emerald/shallow_weaver/slab.h>
#include <emerald/shallow_weaver/state.h>

#include <array>

namespace emerald::shallow_weaver {

class Simulation_old {
public:
    State& _state;

    explicit Simulation_old(Simulation::Parameters const& params,
                            State& _in_state);

    float WaveSpeed = 0.5;

    float WorldSize = 10.0;
    int NX = 64;
    int NY = 64;
    int ArraySize = 64 * 64;
    float DXY = 10.0 / 64;
    float DAMPING = 1.0e-6f;

    enum StateLabel {
        Height,
        TerrainHeight,
        Velocity,
        HeightPrev,
        VelocityPrev,
        VelocityStar,
        AccelerationStar,
        HeightStar,
        JacobiTmp,
        JacobiTmp2
    };

    Float_slab& AState(StateLabel const index);
    Float_slab const& AState(StateLabel const index) const;

    bool InputActive = false;
    int InputIndexX = 0;
    int InputIndexY = 0;
    float InputHeight = 0;

    // 0 for first-order,
    // 1 for RK2
    // 2 for RK4
    int TimeStepMethod = 2;

    // Index an element of a grid in the state array
    int IX(int i, int j) const;
    void EnforceDirichletBoundaryConditions(StateLabel const io_a);
    void EnforceNeumannBoundaryConditions(StateLabel const io_v);
    void EnforceHeightBoundaryConditions(StateLabel const io_h);
    void CopyArray(StateLabel const i_src, StateLabel const o_dst);
    void FillArray(StateLabel const o_a, float i_val);
    void SwapHeight();
    void SwapVel();
    void SwapState();
    void EstimateHeightStar(float i_dt);
    void EstimateVelStar(float i_dt);
    void JacobiComputeBias(StateLabel const i_hStar, StateLabel const o_bias, float i_dt);
    void JacobiIterationAccelBias(StateLabel const i_aOld,
                                  StateLabel const o_aNew,
                                  StateLabel const i_Bias,
                                  float i_dt);
    void JacobiSolveAccel(StateLabel const i_hStar, float i_dt);
    void EstimateAccelStar(float i_dt);
    void AccumulateEstimate(float i_dt);
    void ApplyDamping(float i_dt);
    void TimeStepFirstOrder(float i_dt);
    void TimeStepRK2(float i_dt);
    void TimeStepRK4(float i_dt);
};

}  // namespace emerald::shallow_weaver

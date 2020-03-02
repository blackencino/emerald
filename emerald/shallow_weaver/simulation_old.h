
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

    static constexpr int StateHeight = 0;
    static constexpr int StateVel = 1;
    static constexpr int StateHeightPrev = 2;
    static constexpr int StateVelPrev = 3;
    static constexpr int StateVelStar = 4;
    static constexpr int StateAccelStar = 5;
    static constexpr int StateHeightStar = 6;
    static constexpr int StateJacobiTmp = 7;
    static constexpr int StateJacobiTmp2 = 8;

    Float_slab& AState(int index);
    Float_slab const& AState(int index) const;

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
    void EnforceDirichletBoundaryConditions(int io_a);
    void EnforceNeumannBoundaryConditions(int io_v);
    void EnforceHeightBoundaryConditions(int io_h);
    void CopyArray(int i_src, int o_dst);
    void FillArray(int o_a, float i_val);
    void SwapHeight();
    void SwapVel();
    void SwapState();
    void EstimateHeightStar(float i_dt);
    void EstimateVelStar(float i_dt);
    void JacobiComputeBias(int i_hStar, int o_bias, float i_dt);
    void JacobiIterationAccelBias(int i_aOld,
                                  int o_aNew,
                                  int i_Bias,
                                  float i_dt);
    void JacobiSolveAccel(int i_hStar, float i_dt);
    void EstimateAccelStar(float i_dt);
    void AccumulateEstimate(float i_dt);
    void ApplyDamping(float i_dt);
    void TimeStepFirstOrder(float i_dt);
    void TimeStepRK2(float i_dt);
    void TimeStepRK4(float i_dt);
};

}  // namespace emerald::shallow_weaver

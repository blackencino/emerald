
#include <emerald/wisp/foundation.h>
#include <emerald/wisp/simulation.h>
#include <emerald/wisp/state.h>

#include <array>

namespace emerald::wisp {

class Simulation_old {
public:
    State& _state;

    explicit Simulation_old(Simulation::Parameters const& params,
                            State& _in_state);

    // Grid resolution per side. Rectangular
    int NX = 126;
    int NY = 126;

    // The size of the sim, in "world" units.
    float LX = 100.0f;

    // Size, in "world" units, of a grid cell.
    // Our cells are uniform (square) so DX & DY are the same.
    float DXY;

    // Y size, keeping square cells.
    float LY;

    // The rate at which we inject density
    // into the grid by painting with the
    // mouse.
    float EmissionRate = 2.0f;
    float DenEmissionRadius = 15.0f;
    float VelEmissionRadius = 20.0f;

    // The rate at which density
    // diffuses (dissipates)
    float D_viscosity = 0.00001f;

    // The rate at which velocity
    // dissipates
    float V_viscosity = 0.00001f;

    // The rate at which density decays.
    float D_damp = 0.01f;

    // The rate at which velocity decays.
    float V_damp = 0.0001f;

    // Our time step
    float DT = 1.0f;

    // A scale on input velocity
    float Vscale = 0.75f;

    // Our simulation grids (Our State) will be one cell larger in each
    // dimension to accomodate boundary conditions.
    int GX;
    int GY;

    // The length of all of our (one-dimensional)
    // arrays. We use 1d arrays rather than matrices
    // mostly for efficiency reasons.
    int GridArraySize;

    enum GridSpecifiers : int {
        GridPrevU,
        GridU,
        GridPrevV,
        GridV,
        GridPrevDensityR,
        GridPrevDensityG,
        GridPrevDensityB,
        GridDensityR,
        GridDensityG,
        GridDensityB,
        GridInputU,
        GridInputV,
        GridInputDensityR,
        GridInputDensityG,
        GridInputDensityB,
        GridTemp0,
        GridTemp1,
        GridTemp2
    };

    float VstrokeAlpha = 0.5f;

    //-*****************************************************************************
    // The boundary conditions are enforced on the I selector of the arrays.
    // There are three types of boundary condition application - no negation,
    // just copying at the boundary, then negating in the x-direction only,
    // then negating in the y-direction only.
    static constexpr int BC_NoNegate = 0;
    static constexpr int BC_NegateX = 1;
    static constexpr int BC_NegateY = 2;

    Float_slab& AState(int index);
    Float_slab const& AState(int index) const;

    bool InputActive = false;
    int PrevInputIndexX = 0;
    int PrevInputIndexY = 0;
    int InputIndexX = 0;
    int InputIndexY = 0;
    float InputColorR = 0.0f;
    float InputColorG = 0.0f;
    float InputColorB = 0.0f;

    // Index an element of a grid in the state array
    int IX(int i, int j) const;
    void SwapU();
    void SwapV();
    void SwapVelocity();
    void SwapDensity();
    void SwapArrays();
    void ZeroArray(int i_array);
    void CopyArray(int i_src, int i_dst);
    void GetInputSourceDensity();
    void GetInputSourceVelocity();
    void DampArray(int io_grid, float i_damp);
    void EnforceBoundaryConditions(int io_grid, int i_bType);
    void IntegrateExternalVelocity();
    void IntegrateExternalDensity();
    void Diffuse(int i_OldQ, int o_NewQ, float i_visc, int i_bType);
    void DiffuseDensity();
    void DiffuseVelocity();
    void SemiLagrangianAdvect(
        int i_OldQ, int o_NewQ, int i_GridU, int i_GridV, int i_bType);

    void AdvectDensity();
    void AdvectVelocity();
    void ComputeDivergence(int i_gridU, int i_gridV, int o_gridDiv);

    void ComputePressureViaJacobiIterations(int i_Div,
                                            int o_Pressure,
                                            int i_tmp);
    void ApplyNegativeGradientOfPressureToVelocity(int i_pressure,
                                                   int o_velU,
                                                   int o_velV);
    void EnforceIncompressibility();
    void FluidTimeStep();
};

}  // namespace emerald::wisp

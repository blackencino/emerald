#include <emerald/wisp/simulation_old.h>

#include <emerald/util/functions.h>

namespace emerald::wisp {

Simulation_old::Simulation_old(Simulation::Parameters const& params,
                               State& _in_state)
  : _state(_in_state) {

    NX = params.resolution - 2;
    NY = params.resolution - 2;
    LX = params.world_size;

    DXY = LX / ( float )NX;
    LY = DXY * ( float )NY;

    EmissionRate = params.emission_rate;
    DenEmissionRadius = params.density_emission_radius * 64.0f / float(params.resolution);
    VelEmissionRadius = params.velocity_emission_radius * 64.0f / float(params.resolution);
    D_viscosity = params.density_viscosity;
    V_viscosity = params.velocity_viscosity;
    D_damp = params.density_damping;
    V_damp = params.velocity_damping;
    DT = params.virtual_time_step;
    Vscale = params.velocity_input_scaling;
    GX = NX+2;
    GY = NY+2;
    GridArraySize = GX * GY;

    // Zero out our state to start.
    ZeroArray(GridU);
    ZeroArray(GridV);
    ZeroArray(GridPrevU);
    ZeroArray(GridPrevV);
    ZeroArray(GridPrevDensityR);
    ZeroArray(GridPrevDensityG);
    ZeroArray(GridPrevDensityB);
    ZeroArray(GridDensityR);
    ZeroArray(GridDensityG);
    ZeroArray(GridDensityB);
}

Float_slab& Simulation_old::AState(int index) {
    return reinterpret_cast<Float_slab*>(&_state.PrevU)[index];
}
Float_slab const& Simulation_old::AState(int index) const {
    return reinterpret_cast<Float_slab const*>(&_state.PrevU)[index];
}

// Index an element of a grid in the state array
int Simulation_old::IX(int i, int j) const {
    return (i + GX * j);
}

//-*****************************************************************************
// Swap current arrays (velocity or density) with previous arrays.
void Simulation_old::SwapU() {
    using std::swap;
    swap(AState(GridU), AState(GridPrevU));
}
void Simulation_old::SwapV() {
    using std::swap;
    swap(AState(GridV), AState(GridPrevV));
}

void Simulation_old::SwapVelocity() {
    SwapU();
    SwapV();
}
void Simulation_old::SwapDensity() {
    using std::swap;
    swap(AState(GridDensityR), AState(GridPrevDensityR));
    swap(AState(GridDensityG), AState(GridPrevDensityG));
    swap(AState(GridDensityB), AState(GridPrevDensityB));
}

void Simulation_old::SwapArrays() {
    SwapU();
    SwapV();
    SwapDensity();
}

//-*****************************************************************************
void Simulation_old::ZeroArray(int i_array) {
    for (int a = 0; a < GridArraySize; ++a) { AState(i_array)[a] = 0.0f; }
}

void Simulation_old::CopyArray(int i_src, int i_dst) {
    for (int a = 0; a < GridArraySize; ++a) {
        AState(i_dst)[a] = AState(i_src)[a];
    }
}

//-*****************************************************************************
//-*****************************************************************************
// INPUT
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// Add density based on mouse clicking
//-*****************************************************************************
void Simulation_old::GetInputSourceDensity() {
    if (InputActive) {
        for (int j = 0; j < GY; ++j) {
            float cellMidPointY = (0.5f + (float)j);

            for (int i = 0; i < GX; ++i) {
                float cellMidPointX = (0.5f + (float)i);

                float r = std::hypot(sqr(float(InputIndexX) - cellMidPointX),
                                     sqr(float(InputIndexY) - cellMidPointY));
                float er2 = sqr(2.21f * r / DenEmissionRadius);
                float v = std::clamp(2.0f * std::exp(-er2), 0.0f, 1.0f);

                AState(GridInputDensityR)[IX(i, j)] =
                    InputColorR * EmissionRate * v;
                AState(GridInputDensityG)[IX(i, j)] =
                    InputColorG * EmissionRate * v;
                AState(GridInputDensityB)[IX(i, j)] =
                    InputColorB * EmissionRate * v;
            }
        }
    } else {
        ZeroArray(GridInputDensityR);
        ZeroArray(GridInputDensityG);
        ZeroArray(GridInputDensityB);
    }
}

//-*****************************************************************************
// Derive the velocity of the mouse
// when it's clicked; add the resulting
// velocities to the grid velocity.
//-*****************************************************************************
void Simulation_old::GetInputSourceVelocity() {
    // if ( mousePressed == true && mouseButton == RIGHT )
    if (InputActive) {
        VstrokeAlpha = 0.5f;

        float PixelVelX = float(InputIndexX - PrevInputIndexX) / DT;
        float PixelVelY = float(InputIndexY - PrevInputIndexY) / DT;
        float GridVelX = PixelVelX / 1.0f;
        float GridVelY = PixelVelY / 1.0f;
        float SimVelX = GridVelX * DXY;
        float SimVelY = GridVelY * DXY;

        for (int j = 0; j < GY; ++j) {
            float cellMidPointY = 1.0f * (0.5f + (float)j);

            for (int i = 0; i < GX; ++i) {
                float cellMidPointX = 1.0f * (0.5f + (float)i);

                float r = std::hypot(sqr(float(InputIndexX) - cellMidPointX),
                                     sqr(float(InputIndexY) - cellMidPointY));
                float er2 = sqr(2.21f * r / VelEmissionRadius);
                float v = std::clamp(2.0f * std::exp(-er2), 0.0f, 1.0f);

                AState(GridInputU)[IX(i, j)] = SimVelX * Vscale * v;
                AState(GridInputV)[IX(i, j)] = SimVelY * Vscale * v;
            }
        }
    } else {
        ZeroArray(GridInputU);
        ZeroArray(GridInputV);
    }
}

//-*****************************************************************************
//-*****************************************************************************
// PHYSICS FUNCTIONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// Damping
void Simulation_old::DampArray(int io_grid, float i_damp) {
    float mult = std::pow(std::clamp(1.0f - i_damp, 0.0f, 1.0f), DT);
    for (int a = 0; a < GridArraySize; ++a) { AState(io_grid)[a] *= mult; }
}

//-*****************************************************************************
// The boundary conditions are enforced on the I selector of the arrays.
// There are three types of boundary condition application - no negation,
// just copying at the boundary, then negating in the x-direction only,
// then negating in the y-direction only.
int BC_NoNegate = 0;
int BC_NegateX = 1;
int BC_NegateY = 2;

//-*****************************************************************************
void Simulation_old::EnforceBoundaryConditions(int io_grid, int i_bType) {
    // Copy the bottom row from the one above it. If the boundary type
    // is '2', that means negate the values.
    // Do the same for the topmost row and the one beneath it.
    for (int i = 1; i <= NX; ++i) {
        AState(io_grid)[IX(i, 0)] = (i_bType == BC_NegateY)
                                        ? -AState(io_grid)[IX(i, 1)]
                                        : AState(io_grid)[IX(i, 1)];

        AState(io_grid)[IX(i, NY + 1)] = (i_bType == BC_NegateY)
                                             ? -AState(io_grid)[IX(i, NY)]
                                             : AState(io_grid)[IX(i, NY)];
    }

    // Copy the left col from the one to the right of it. If the boundary type
    // is '1', that means negate the values.
    // Do the same for the rightmost col and the one to the left of it.
    for (int j = 1; j <= NY; ++j) {
        AState(io_grid)[IX(0, j)] = (i_bType == BC_NegateX)
                                        ? -AState(io_grid)[IX(1, j)]
                                        : AState(io_grid)[IX(1, j)];

        AState(io_grid)[IX(NX + 1, j)] = (i_bType == BC_NegateY)
                                             ? -AState(io_grid)[IX(NX, j)]
                                             : AState(io_grid)[IX(NX, j)];
    }

    // Get each corner by averaging the two boundary values adjacent.
    AState(io_grid)[IX(0, 0)] =
        0.5f * (AState(io_grid)[IX(1, 0)] + AState(io_grid)[IX(0, 1)]);
    AState(io_grid)[IX(0, NY + 1)] =
        0.5f * (AState(io_grid)[IX(1, NY + 1)] + AState(io_grid)[IX(0, NY)]);
    AState(io_grid)[IX(NX + 1, 0)] =
        0.5f * (AState(io_grid)[IX(NX, 0)] + AState(io_grid)[IX(NX + 1, 1)]);
    AState(io_grid)[IX(NX + 1, NY + 1)] =
        0.5f *
        (AState(io_grid)[IX(NX, NY + 1)] + AState(io_grid)[IX(NX + 1, NY)]);
}

//-*****************************************************************************
// Integrate External Forces (basically, in this case, just add the
// input source velocity to the velocity)
void Simulation_old::IntegrateExternalVelocity() {
    // We can work directly on final density.
    for (int a = 0; a < GridArraySize; ++a) {
        AState(GridU)[a] += DT * AState(GridInputU)[a];
        AState(GridV)[a] += DT * AState(GridInputV)[a];
    }

    EnforceBoundaryConditions(GridU, BC_NegateX);
    EnforceBoundaryConditions(GridV, BC_NegateY);
}

//-*****************************************************************************
// Integrate External Densities (basically, in this case, just add the
// input source density to the density)
void Simulation_old::IntegrateExternalDensity() {
    if (InputActive) {
        // We can work directly on final density.
        for (int a = 0; a < GridArraySize; ++a) {
            AState(GridDensityR)[a] += DT * AState(GridInputDensityR)[a];
            AState(GridDensityG)[a] += DT * AState(GridInputDensityG)[a];
            AState(GridDensityB)[a] += DT * AState(GridInputDensityB)[a];
        }

        EnforceBoundaryConditions(GridDensityR, BC_NoNegate);
        EnforceBoundaryConditions(GridDensityG, BC_NoNegate);
        EnforceBoundaryConditions(GridDensityB, BC_NoNegate);
    }
}

//-*****************************************************************************
// A simple equation for diffusion is the heat equation:
// dQ/dt = DiffusionRate * Laplacian( Q )
// which you can read more about here:
// http://en.wikipedia.org/wiki/Heat_equation
//
// As in the jacobi pressure solver below, we solve this equation by
// discretizing these differential operators and then solving for the
// central value, assuming all other values are constant.
//
// ( Qcen - QcenOld ) / Dt = DiffusionRate * Laplacian( Q )
// ( Qcen - QcenOld ) - Dt * DiffusionRate * Laplacian( Q ) = 0
// k = Dt * DiffusionRate / DXY^2
// Qcen - QcenOld - k * (( Qdown + Qleft + Qright + Qup ) - 4.0Qcen) = 0
// (1 - 4*k)*Qcen - QcenOld - k*(Qdown + Qleft + Qright + Qup) = 0
// (1 - 4*k)*Qcen = QcenOld + k*(Qdown + Qleft + Qrigth + Qup)
// Qcen = (QcenOld + k*(Qdown + Qleft + Qright + Qup)) / ( 1 - 4*k )
// --------------------------------
// Diffuse (dissipate) density
// --------------------------------
void Simulation_old::Diffuse(int i_OldQ,
                             int o_NewQ,
                             float i_visc,
                             int i_bType) {
    float k = DT * i_visc * sqr(DXY);
    // print( "k = " + k );

    // Create temporary handles to src and dst arrays, which
    // we will ping-pong.
    int SRC = o_NewQ;
    int DST = i_OldQ;

    for (int iters = 0; iters < 9; ++iters) {
        // Swap src and dst array pointers.
        using std::swap;
        swap(AState(SRC), AState(DST));

        // Diffuse the SRC into DST.
        for (int j = 1; j <= NY; ++j) {
            for (int i = 1; i <= NX; ++i) {
                AState(DST)[IX(i, j)] =
                    (AState(SRC)[IX(i, j)] + k * (AState(SRC)[IX(i, j - 1)] +
                                                  AState(SRC)[IX(i - 1, j)] +
                                                  AState(SRC)[IX(i + 1, j)] +
                                                  AState(SRC)[IX(i, j + 1)])) /
                    (1.0f + 4.0f * k);
            }
        }

        // Enforce the boundary conditions.
        EnforceBoundaryConditions(DST, i_bType);
    }
}

//-*****************************************************************************
void Simulation_old::DiffuseDensity() {
    SwapDensity();
    Diffuse(GridPrevDensityR, GridDensityR, D_viscosity, BC_NoNegate);
    Diffuse(GridPrevDensityG, GridDensityG, D_viscosity, BC_NoNegate);
    Diffuse(GridPrevDensityB, GridDensityB, D_viscosity, BC_NoNegate);
}

//-*****************************************************************************
void Simulation_old::DiffuseVelocity() {
    SwapVelocity();
    Diffuse(GridPrevU, GridU, V_viscosity, BC_NegateX);
    Diffuse(GridPrevV, GridV, V_viscosity, BC_NegateY);
}

//-*****************************************************************************
void Simulation_old::SemiLagrangianAdvect(
    int i_OldQ, int o_NewQ, int i_GridU, int i_GridV, int i_bType) {
    for (int j = 1; j <= NY; ++j) {
        float SimPosY = DXY * (0.5f + (float)j);
        for (int i = 1; i <= NX; ++i) {
            float SimPosX = DXY * (0.5f + (float)i);

            float SimVelX = AState(i_GridU)[IX(i, j)];
            float SimVelY = AState(i_GridV)[IX(i, j)];

            float SimSamplePosX = SimPosX - DT * SimVelX;
            float SimSamplePosY = SimPosY - DT * SimVelY;

            float GridSamplePosX = (SimSamplePosX / DXY) - 0.5f;
            float GridSamplePosY = (SimSamplePosY / DXY) - 0.5f;

            int MinI = (int)std::floor(GridSamplePosX);
            float InterpU = GridSamplePosX - (float)MinI;
            MinI = std::clamp(MinI, 0, GX - 1);

            int MinJ = (int)std::floor(GridSamplePosY);
            float InterpV = GridSamplePosY - (float)MinJ;
            MinJ = std::clamp(MinJ, 0, GY - 1);

            int MaxI = std::clamp(MinI + 1, 0, GX - 1);
            int MaxJ = std::clamp(MinJ + 1, 0, GY - 1);

            float Q00 = AState(i_OldQ)[IX(MinI, MinJ)];
            float Q10 = AState(i_OldQ)[IX(MaxI, MinJ)];
            float Q01 = AState(i_OldQ)[IX(MinI, MaxJ)];
            float Q11 = AState(i_OldQ)[IX(MaxI, MaxJ)];

            float Qdown = mix(Q00, Q10, InterpU);
            float Qup = mix(Q01, Q11, InterpU);

            AState(o_NewQ)[IX(i, j)] = mix(Qdown, Qup, InterpV);
        }
    }
    Simulation_old::EnforceBoundaryConditions(o_NewQ, i_bType);
}

//-*****************************************************************************
void Simulation_old::AdvectDensity() {
    SwapDensity();
    SemiLagrangianAdvect(
        GridPrevDensityR, GridDensityR, GridU, GridV, BC_NoNegate);
    SemiLagrangianAdvect(
        GridPrevDensityG, GridDensityG, GridU, GridV, BC_NoNegate);
    SemiLagrangianAdvect(
        GridPrevDensityB, GridDensityB, GridU, GridV, BC_NoNegate);
}

//-*****************************************************************************
void Simulation_old::AdvectVelocity() {
    SwapVelocity();
    SemiLagrangianAdvect(GridPrevU, GridU, GridPrevU, GridPrevV, BC_NegateX);
    SemiLagrangianAdvect(GridPrevV, GridV, GridPrevU, GridPrevV, BC_NegateY);
}

//-*****************************************************************************
// Compute Divergence.
//
// Compute the divergence. This is the amount of mass entering
// or exiting each cell. In an incompressible fluid, divergence is
// zero - and that's what we're solving for!
//
// It is equal to the partial derivative of the x-velocity in the x direction
// (dU/dx),
// plus the partial derivative of the y-velocity in the y direction
// (dV/dy).
// We're using a central differencing scheme for computing the derivatives
// here.
//-*****************************************************************************
void Simulation_old::ComputeDivergence(int i_gridU,
                                       int i_gridV,
                                       int o_gridDiv) {
    for (int j = 1; j <= NY; ++j) {
        for (int i = 1; i <= NX; ++i) {
            float twoDU =
                AState(i_gridU)[IX(i + 1, j)] - AState(i_gridU)[IX(i - 1, j)];
            float twoDV =
                AState(i_gridV)[IX(i, j + 1)] - AState(i_gridV)[IX(i, j - 1)];
            AState(o_gridDiv)[IX(i, j)] =
                (twoDU / (2.0f * DXY)) + (twoDV / (2.0f * DXY));
        }
    }

    // Compute Divergence Boundary conditions.
    EnforceBoundaryConditions(o_gridDiv, BC_NoNegate);
}

//-*****************************************************************************
// We're using the Projection Method for solving an incompressible flow.
//
// Information about this method can be found here:
// http://en.wikipedia.org/wiki/Projection_method_(fluid_dynamics)
//
// But, essentially, what it means is that we're breaking the solving
// of the equation up into separate steps, roughly:
// Add External Forces
// Advect
// Compute Divergence
// Find Pressure Which Will Correct Divergence
// Adjust Velocity By Negative Gradient Of Pressure.
//
// This function implements the Find Pressure part, by iteratively
// solving for a pressure which satisifies (at every point in the grid)
// the relationship:  Laplacian( Pressure ) = Divergence.
// The Laplacian is a differential operator which is equal to the
// second partial derivative with respect to x plus the second partial
// derivative with respect to y.  This can be discretized in the following
// way:
//
// SecondDerivX( P, i, j ) = ( ( P[i+1,j] - P[i,j] ) / DXY ) -
//                             ( P[i,j] - P[i-1,j] ) / DXY ) ) / DXY
// SecondDerivY( P, i, j ) = ( ( P[i,j+1] - P[i,j] ) / DXY ) -
//                             ( P[i,j] - P[i,j-1] ) / DXY ) ) / DXY
//
// Laplacian( P, i, j ) = SecondDerivX( P, i, j ) + SecondDerivY( P, i, j )
//
// This simplifies to:
// (( P[i,j-1] + P[i-1,j] + P[i+1,j] + P[i,j+1] ) - 4 P[i,j] ) / DXY^2
//
// The idea with jacobi iteration is to assume, at every i,j point, that
// all the other values are constant, and just solve for one's self.
// This needs to be done into a new destination array, otherwise you'll be
// writing over your data as you're computing it. There are simulation
// techniques such as red-black gauss-seidel which can compute the data
// in-place, but we'll just swap between an old and a new pressure.
//
// Given the above expression for the lapacian at i,j, and the relationship
// Laplacian( P ) = Divergence
// We set the above expression to Divergence[i,j], and then rearrange the
// equation so that we're solving for P[i,j]:
//
// (( Pdown + Pleft + Pright + Pup ) - 4 Pcen )/H^2 = Div
// ( Pdown + Pleft + Pright + Pup ) - 4 Pcen  = H^2 * Div
// -4 Pcen = ( H^2 * Div ) - ( Pdown + Pleft + Pright + Pup )
// 4 Pcen = ( Pdown + Pleft + Pright + Pup ) - ( H^2 * Div )
// Pcen = ( ( Pdown + Pleft + Pright + Pup ) - ( H^2 * Div ) ) / 4
//
// P[i,j] = -( DXY^2 * Divergence[i,j] +
//          (( P[i,j-1] + P[i-1,j] + P[i+1,j] + P[i,j+1] )) ) / 4
//
//-*****************************************************************************
void Simulation_old::ComputePressureViaJacobiIterations(int i_Div,
                                                        int o_Pressure,
                                                        int i_tmp) {
    // Init array indices.
    int SRC = o_Pressure;
    int DST = i_tmp;

    // Init the DST pressure to zero. It will be swapped into the SRC
    // location in the loop below.
    ZeroArray(DST);

    // Iterate 20 times, improving the pressure current from the pressure
    // prev.
    for (int iter = 0; iter < 10; ++iter) {
        using std::swap;
        swap(AState(SRC), AState(DST));

        // Do a single jacobi iteration to compute the current pressure
        // from the previous pressure.
        for (int j = 1; j <= NY; ++j) {
            for (int i = 1; i <= NX; ++i) {
                AState(DST)[IX(i, j)] =
                    ((AState(SRC)[IX(i, j - 1)] + AState(SRC)[IX(i - 1, j)] +
                      AState(SRC)[IX(i + 1, j)] + AState(SRC)[IX(i, j + 1)]) -
                     (DXY * DXY * AState(i_Div)[IX(i, j)])) /
                    4.0f;
            }
        }

        // Okay we've solved for DST. Enforce boundary conditions on it,
        // without negating in any direction.
        EnforceBoundaryConditions(DST, BC_NoNegate);
    }
}

//-*****************************************************************************
// Apply Negative Gradient of Pressure to Velocity
void Simulation_old::ApplyNegativeGradientOfPressureToVelocity(int i_pressure,
                                                               int o_velU,
                                                               int o_velV) {
    for (int j = 1; j <= NY; ++j) {
        for (int i = 1; i <= NX; ++i) {
            float twoDPx = AState(i_pressure)[IX(i + 1, j)] -
                           AState(i_pressure)[IX(i - 1, j)];
            float twoDPy = AState(i_pressure)[IX(i, j + 1)] -
                           AState(i_pressure)[IX(i, j - 1)];

            AState(o_velU)[IX(i, j)] -= twoDPx / (2.0f * DXY);
            AState(o_velV)[IX(i, j)] -= twoDPy / (2.0f * DXY);
        }
    }

    // And apply boundary conditions. The U velocities are negated horizonally,
    // and the V velocities are negated vertically. This makes the fluid
    // reflect off the boundaries.
    EnforceBoundaryConditions(o_velU, BC_NegateX);
    EnforceBoundaryConditions(o_velV, BC_NegateY);
}

//-*****************************************************************************
void Simulation_old::EnforceIncompressibility() {
    int DIV = GridTemp0;
    int PRES = GridTemp1;
    int TMP = GridTemp2;
    ComputeDivergence(GridU, GridV, DIV);
    ComputePressureViaJacobiIterations(DIV, PRES, TMP);
    ApplyNegativeGradientOfPressureToVelocity(PRES, GridU, GridV);
}

//-*****************************************************************************
void Simulation_old::FluidTimeStep() {
    // Get External Input
    GetInputSourceDensity();
    GetInputSourceVelocity();

    // Solve Velocities
    AdvectVelocity();
    DampArray(GridU, V_damp);
    DampArray(GridV, V_damp);
    DiffuseVelocity();
    IntegrateExternalVelocity();
    EnforceIncompressibility();
    // AdvectFluid();
    // EnforceIncompressibility();

    // Solve Densities
    AdvectDensity();
    DiffuseDensity();
    DampArray(GridDensityR, D_damp);
    DampArray(GridDensityG, D_damp);
    DampArray(GridDensityB, D_damp);
    IntegrateExternalDensity();
    // DiffuseDensities();
    // IntegrateExternalDensities();
    // AdvectDensities();
}

}  // namespace emerald::wisp

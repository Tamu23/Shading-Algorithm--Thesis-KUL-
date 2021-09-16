within ;
package BEM

  package ConstructionComponents
    record ExternalWall "External Wall (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        mats={IDEAS.Buildings.Data.Materials.Glass(k=0.86,c=780, rho=2450, d=0.004, epsLw=0.87, epsSw=0.70),
              IDEAS.Buildings.Data.Materials.Air(d=0.04),
              IDEAS.Buildings.Data.Insulation.Rockwool(d=0.16),
              IDEAS.Buildings.Data.Materials.Plywood(d=0.01)});

      annotation (Documentation(info="<html>
<p>
Example implementation of a cavity wall.
</p>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>"));
    end ExternalWall;

    record InternalWall "Internal Wall (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        mats={IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17),
              IDEAS.Buildings.Data.Insulation.Glasswool(k=0.045,c=1450, rho=15, d=0.05),
              IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17)});

    end InternalWall;

    record Floor_Ceil
      "Floor_Ceil Simplified for single zone (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        locGain={1},
        incLastLay = IDEAS.Types.Tilt.Floor,
        mats={IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17),
        IDEAS.Buildings.Data.Materials.Concrete(k=1.7, c=1000, rho=2400, d=0.25,epsLw=0.85,epsSw=0.6),
        IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17),
        IDEAS.Buildings.Data.Materials.Plywood(k=0.24,c=1000, rho=800, d=0.04),
            IDEAS.Buildings.Data.Materials.Timber(k=0.06,c=1300, rho=200, d=0.01,epsLw=0.85)});

    annotation (Documentation(info="<html>
<p>
Example implementation of a Thermally Activated Building System.
</p>
</html>",     revisions="<html>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>"));
    end Floor_Ceil;

    record Structure "Concrete Layer (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        locGain={1},
        incLastLay = IDEAS.Types.Tilt.Ceiling,
        mats={IDEAS.Buildings.Data.Materials.Concrete(k=1.7, c=1000, rho=2400, d=0.25,epsLw=0.85, epsSw=0.6)});

    annotation (Documentation(info="<html>
<p>
Example implementation of a Thermally Activated Building System.
</p>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>",     revisions="<html>
<ul>
<li>
March 7, 2017 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"));
    end Structure;

    record SuspCeiling "Suspended ceiling (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        locGain={1},
        incLastLay = IDEAS.Types.Tilt.Ceiling,
        mats={IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17)});

    annotation (Documentation(info="<html>
<p>
Example implementation of a Thermally Activated Building System.
</p>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>",     revisions="<html>
<ul>
<li>
March 7, 2017 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"));
    end SuspCeiling;

    record ElevFloor "Elevated floor (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        locGain={1},
        incLastLay = IDEAS.Types.Tilt.Floor,
        mats={IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17),
        IDEAS.Buildings.Data.Materials.Plywood(k=0.24,c=1000, rho=800, d=0.04),
            IDEAS.Buildings.Data.Materials.Timber(k=0.06,c=1300, rho=200, d=0.01,epsLw=0.85)});

    annotation (Documentation(info="<html>
<p>
Example implementation of a Thermally Activated Building System.
</p>
</html>",     revisions="<html>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>"));
    end ElevFloor;

    record ElevFloor1 "Elevated floor with concrete (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        locGain={1},
        incLastLay = IDEAS.Types.Tilt.Floor,
        mats={IDEAS.Buildings.Data.Materials.Concrete(k=1.7, c=1000, rho=2400, d=0.25,epsLw=0.85, epsSw=0.6),
        IDEAS.Buildings.Data.Materials.Gypsum(k=0.25,c=1050, rho=900, d=0.015,epsSw=0.17),
        IDEAS.Buildings.Data.Materials.Plywood(k=0.24,c=1000, rho=800, d=0.04),
            IDEAS.Buildings.Data.Materials.Timber(k=0.06,c=1300, rho=200, d=0.01,epsLw=0.85)});

    annotation (Documentation(info="<html>
<p>
Example implementation of a Thermally Activated Building System.
</p>
</html>",     revisions="<html>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>"));
    end ElevFloor1;

    model Window "Multipane window (with shading factor import added)"
      replaceable IDEAS.Buildings.Data.Interfaces.Glazing glazing
        constrainedby IDEAS.Buildings.Data.Interfaces.Glazing "Glazing type"
        annotation (__Dymola_choicesAllMatching=true, Dialog(group=
              "Construction details"));

      extends IDEAS.Buildings.Components.Interfaces.PartialSurface(
        dT_nominal_a=-3,
        intCon_a(final A=
               A*(1 - frac),
               linearise=linIntCon_a or sim.linearise,
               dT_nominal=dT_nominal_a),
        QTra_design(fixed=false),
        Qgai(y=if sim.computeConservationOfEnergy then
                                                      (gain.propsBus_a.surfCon.Q_flow +
            gain.propsBus_a.surfRad.Q_flow + gain.propsBus_a.iSolDif.Q_flow + gain.propsBus_a.iSolDir.Q_flow) else 0),
        E(y=0),
        layMul(
          A=A*(1 - frac),
          nLay=glazing.nLay,
          mats=glazing.mats,
          energyDynamics=if windowDynamicsType == IDEAS.Buildings.Components.Interfaces.WindowDynamicsType.Normal then energyDynamics else Modelica.Fluid.Types.Dynamics.SteadyState,
          dT_nom_air=5,
          linIntCon=true,
          checkCoatings=glazing.checkLowPerformanceGlazing));
      parameter Boolean linExtCon=sim.linExtCon
        "= true, if exterior convective heat transfer should be linearised (uses average wind speed)"
        annotation(Dialog(tab="Convection"));
      parameter Boolean linExtRad=sim.linExtRadWin
        "= true, if exterior radiative heat transfer should be linearised"
        annotation(Dialog(tab="Radiation"));

      parameter Real frac(
        min=0,
        max=1) = 0.15 "Area fraction of the window frame";
      parameter IDEAS.Buildings.Components.Interfaces.WindowDynamicsType
        windowDynamicsType=IDEAS.Buildings.Components.Interfaces.WindowDynamicsType.Two
        "Type of dynamics for glazing and frame: using zero, one combined or two states"
        annotation (Dialog(tab="Dynamics", group="Equations", enable = not energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState));
      parameter Real fraC = frac
        "Ratio of frame and glazing thermal masses"
        annotation(Dialog(tab="Dynamics", group="Equations", enable= not energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState and windowDynamicsType == IDEAS.Buildings.Components.Interfaces.WindowDynamicsType.Two));
      parameter Boolean controlled = shaType.controlled
        " = true if shaType has a control input (see e.g. screen). Can be set to false manually to force removal of input icon."
        annotation(Dialog(tab="Advanced",group="Icon"));

      replaceable parameter IDEAS.Buildings.Data.Frames.None fraType
        constrainedby IDEAS.Buildings.Data.Interfaces.Frame "Window frame type"
        annotation (choicesAllMatching=true, Dialog(group=
              "Construction details"));
      replaceable IDEAS.Buildings.Components.Shading.None shaType constrainedby
        IDEAS.Buildings.Components.Shading.Interfaces.PartialShading(
                                final azi=aziInt) "First shading type"  annotation (Placement(transformation(extent={{-70,-60},
                {-60,-40}})),
          __Dymola_choicesAllMatching=true, Dialog(group="Construction details"));

      Modelica.Blocks.Interfaces.RealInput Ctrl if controlled
        "Control signal between 0 and 1, i.e. 1 is fully closed" annotation (
          Placement(transformation(
            extent={{20,-20},{-20,20}},
            rotation=-90,
            origin={-50,-110}), iconTransformation(
            extent={{10,-10},{-10,10}},
            rotation=-90,
            origin={-40,-100})));

      DirFac dirFac(shDir=true)
        annotation (Placement(transformation(extent={{-70,-74},{-62,-66}})));
      SkyDifFac skyDifFac(shDif=true)
        annotation (Placement(transformation(extent={{-80,-84},{-72,-76}})));
    protected
      final parameter Real U_value=glazing.U_value*(1-frac)+fraType.U_value*frac
        "Average window U-value";
      final parameter Boolean addCapGla =  windowDynamicsType == IDEAS.Buildings.Components.Interfaces.WindowDynamicsType.Two and not energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState
        "Add lumped thermal capacitor for window glazing";
      final parameter Boolean addCapFra =  fraType.present and not energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState
        "Added lumped thermal capacitor for window frame";
      final parameter Modelica.SIunits.HeatCapacity Cgla = layMul.C
        "Heat capacity of glazing state";
      final parameter Modelica.SIunits.HeatCapacity Cfra = layMul.C*fraC
        "Heat capacity of frame state";

      IDEAS.Buildings.Components.BaseClasses.ConvectiveHeatTransfer.ExteriorConvection
        eCon(
        final A=A*(1 - frac),
        linearise=linExtCon or sim.linearise,
        final inc=incInt,
        final azi=aziInt)
        "Convective surface heat transimission on the exterior side of the wall"
        annotation (Placement(transformation(extent={{-20,-38},{-40,-18}})));

      IDEAS.Buildings.Components.BaseClasses.RadiativeHeatTransfer.ExteriorHeatRadiation
        skyRad(final A=A*(1 - frac), Tenv_nom=sim.Tenv_nom,
        linearise=linExtRad or sim.linearise)
        "determination of radiant heat exchange with the environment and sky"
        annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
      replaceable
      IDEAS.Buildings.Components.BaseClasses.RadiativeHeatTransfer.SwWindowResponse
        solWin(
        final nLay=glazing.nLay,
        final SwAbs=glazing.SwAbs,
        final SwTrans=glazing.SwTrans,
        final SwTransDif=glazing.SwTransDif,
        final SwAbsDif=glazing.SwAbsDif)
        annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));

      IDEAS.Buildings.Components.BaseClasses.ConvectiveHeatTransfer.InteriorConvection
        iConFra(final A=A*frac, final inc=incInt,
        linearise=linIntCon_a or sim.linearise) if
                            fraType.present
        "convective surface heat transimission on the interior side of the wall"
        annotation (Placement(transformation(extent={{20,60},{40,80}})));
      IDEAS.Buildings.Components.BaseClasses.RadiativeHeatTransfer.ExteriorHeatRadiation
        skyRadFra(final A=A*frac, Tenv_nom=sim.Tenv_nom,
        linearise=linExtRad or sim.linearise) if
                             fraType.present
        "determination of radiant heat exchange with the environment and sky"
        annotation (Placement(transformation(extent={{-20,80},{-40,100}})));
      IDEAS.Buildings.Components.BaseClasses.ConvectiveHeatTransfer.ExteriorConvection
        eConFra(final A=A*frac, linearise=linExtCon or sim.linearise,
        inc=incInt,
        azi=aziInt) if
                     fraType.present
        "convective surface heat transimission on the exterior side of the wall"
        annotation (Placement(transformation(extent={{-20,60},{-40,80}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor layFra(final G=(if
            fraType.briTyp.present then fraType.briTyp.G else 0) + (fraType.U_value)
            *A*frac) if                fraType.present  annotation (Placement(transformation(extent={{10,60},
                {-10,80}})));

      IDEAS.BoundaryConditions.SolarIrradiation.RadSolData radSolData(
        inc=incInt,
        azi=aziInt,
        lat=sim.lat,
        outputAngles=sim.outputAngles,
        incAndAziInBus=sim.incAndAziInBus,
        numIncAndAziInBus=sim.numIncAndAziInBus)
        annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));
      Modelica.Blocks.Math.Gain gainDir(k=A*(1 - frac))
        "Gain for direct solar irradiation"
        annotation (Placement(transformation(extent={{-42,-46},{-38,-42}})));
      Modelica.Blocks.Math.Gain gainDif(k=A*(1 - frac))
        "Gain for diffuse solar irradiation"
        annotation (Placement(transformation(extent={{-36,-50},{-32,-46}})));
      Modelica.Blocks.Routing.RealPassThrough Tdes
        "Design temperature passthrough since propsBus variables cannot be addressed directly";
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCapGlaInt(C=Cgla/2,
          T(fixed=energyDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial,
            start=T_start)) if                                                                             addCapGla
        "Heat capacitor for glazing at interior"
        annotation (Placement(transformation(extent={{6,-12},{26,-32}})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCapFraIn(C=Cfra/2,
          T(fixed=energyDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial,
            start=T_start)) if                                                                             addCapFra
        "Heat capacitor for frame at interior"
        annotation (Placement(transformation(extent={{4,100},{24,120}})));
      Modelica.Blocks.Sources.Constant constEpsSwFra(final k=fraType.mat.epsSw)
        "Shortwave emissivity of frame"
        annotation (Placement(transformation(extent={{4,46},{-6,56}})));
      Modelica.Blocks.Sources.Constant constEpsLwFra(final k=fraType.mat.epsLw)
        "Shortwave emissivity of frame"
        annotation (Placement(transformation(extent={{4,86},{-6,96}})));
      IDEAS.Buildings.Components.BaseClasses.RadiativeHeatTransfer.ExteriorSolarAbsorption
        solAbs(A=A*frac) if fraType.present
        "Solar absorption model for shortwave radiation"
        annotation (Placement(transformation(extent={{-20,40},{-40,60}})));
      Modelica.Blocks.Math.Add solDif(final k1=1, final k2=1)
        "Sum of ground and sky diffuse solar irradiation"
        annotation (Placement(transformation(extent={{-56,-50},{-50,-44}})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCapFraExt(C=Cfra/2,
          T(fixed=energyDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial,
            start=T_start)) if                                                                             addCapFra
        "Heat capacitor for frame at exterior"
        annotation (Placement(transformation(extent={{-20,100},{0,120}})));
      Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCapGlaExt(C=Cgla/2,
          T(fixed=energyDynamics == Modelica.Fluid.Types.Dynamics.FixedInitial,
            start=T_start)) if                                                                             addCapGla
        "Heat capacitor for glazing at exterior"
        annotation (Placement(transformation(extent={{-20,-12},{0,-32}})));
    initial equation
      QTra_design = (U_value*A + (if fraType.briTyp.present then fraType.briTyp.G else 0)) *(273.15 + 21 - Tdes.y);

    equation
      connect(eCon.port_a, layMul.port_b) annotation (Line(
          points={{-20,-28},{-14,-28},{-14,0},{-10,0}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(skyRad.port_a, layMul.port_b) annotation (Line(
          points={{-20,0},{-10,0}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(solWin.iSolDir, propsBusInt.iSolDir) annotation (Line(
          points={{-2,-60},{-2,-70},{56.09,-70},{56.09,19.91}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(solWin.iSolDif, propsBusInt.iSolDif) annotation (Line(
          points={{2,-60},{2,-70},{56.09,-70},{56.09,19.91}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(solWin.iSolAbs, layMul.port_gain) annotation (Line(
          points={{0,-40},{0,-10}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(layMul.iEpsLw_b, skyRad.epsLw) annotation (Line(
          points={{-10,8},{-14,8},{-14,3.4},{-20,3.4}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(shaType.Ctrl, Ctrl) annotation (Line(
          points={{-65,-60},{-50,-60},{-50,-110}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(iConFra.port_b, propsBusInt.surfCon) annotation (Line(
          points={{40,70},{46,70},{46,19.91},{56.09,19.91}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(layFra.port_a, iConFra.port_a) annotation (Line(
          points={{10,70},{20,70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(skyRadFra.port_a, layFra.port_b) annotation (Line(
          points={{-20,90},{-16,90},{-16,70},{-10,70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(eConFra.port_a, layFra.port_b) annotation (Line(
          points={{-20,70},{-10,70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(radSolData.angInc, shaType.angInc) annotation (Line(
          points={{-79.4,-54},{-76,-54},{-76,-54},{-70,-54}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(radSolData.angAzi, shaType.angAzi) annotation (Line(
          points={{-79.4,-58},{-76,-58},{-76,-58},{-70,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(radSolData.angZen, shaType.angZen) annotation (Line(
          points={{-79.4,-56},{-76,-56},{-76,-56},{-70,-56}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(radSolData.weaBus, propsBusInt.weaBus) annotation (Line(
          points={{-80,-42},{-80,20},{0,20},{0,19.91},{56.09,19.91}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(radSolData.Tenv, skyRad.Tenv) annotation (Line(
          points={{-79.4,-52},{-72,-52},{-72,10},{-20,10},{-20,6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(skyRadFra.Tenv, skyRad.Tenv) annotation (Line(
          points={{-20,96},{-12,96},{-12,6},{-20,6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(eConFra.Te, eCon.Te) annotation (Line(
          points={{-20,65.2},{-20,66},{-16,66},{-16,-32.8},{-20,-32.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(eCon.hForcedConExt, eConFra.hForcedConExt) annotation (Line(
          points={{-20,-37},{-20,-36},{-14,-36},{-14,61},{-20,61}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(eCon.Te, propsBusInt.weaBus.Te) annotation (Line(
          points={{-20,-32.8},{56.09,-32.8},{56.09,19.91}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(Tdes.u, propsBusInt.weaBus.Tdes);
      connect(shaType.iAngInc, solWin.angInc) annotation (Line(points={{-60,-54},{
              -60,-54},{-10,-54}},           color={0,0,127}));
      connect(heaCapGlaInt.port, layMul.port_a)
        annotation (Line(points={{16,-12},{16,0},{10,0}}, color={191,0,0}));
      connect(heaCapFraIn.port, layFra.port_a)
        annotation (Line(points={{14,100},{14,70},{10,70}}, color={191,0,0}));
      connect(skyRadFra.epsLw, constEpsLwFra.y) annotation (Line(points={{-20,93.4},
              {-14,93.4},{-14,91},{-6.5,91}}, color={0,0,127}));
      connect(solAbs.port_a, layFra.port_b) annotation (Line(points={{-20,50},{-16,
              50},{-16,70},{-10,70}},
                                  color={191,0,0}));
      connect(solAbs.epsSw, constEpsSwFra.y) annotation (Line(points={{-20,56},{-10,
              56},{-10,51},{-6.5,51}}, color={0,0,127}));
      connect(gainDir.y, solWin.solDir)
        annotation (Line(points={{-37.8,-44},{-10,-44}}, color={0,0,127}));
      connect(gainDif.y, solWin.solDif) annotation (Line(points={{-31.8,-48},{-22,
              -48},{-10,-48}}, color={0,0,127}));
      connect(radSolData.HGroDifTil, shaType.HGroDifTil) annotation (Line(points={{-79.4,
              -50},{-74,-50},{-74,-48},{-70,-48}},       color={0,0,127}));
      connect(shaType.HShaGroDifTil, solDif.u2) annotation (Line(points={{-60,-48},
              {-56.6,-48},{-56.6,-48.8}}, color={0,0,127}));
      connect(solDif.u1, shaType.HShaSkyDifTil) annotation (Line(points={{-56.6,
              -45.2},{-56.3,-45.2},{-56.3,-46},{-60,-46}}, color={0,0,127}));
      connect(gainDif.u, solDif.y) annotation (Line(points={{-36.4,-48},{-49.7,-48},
              {-49.7,-47}}, color={0,0,127}));
      connect(solDif.y, solAbs.solDif) annotation (Line(points={{-49.7,-47},{-48,
              -47},{-48,52},{-40,52}}, color={0,0,127}));
      connect(shaType.HShaDirTil, solAbs.solDir) annotation (Line(points={{-60,-44},
              {-60,-44},{-60,56},{-40,56}}, color={0,0,127}));
      connect(gainDir.u, shaType.HShaDirTil) annotation (Line(points={{-42.4,-44},{
              -51.2,-44},{-60,-44}}, color={0,0,127}));
      connect(eCon.hForcedConExt, radSolData.hForcedConExt) annotation (Line(points=
             {{-20,-37},{-50,-37},{-50,-62.2},{-79.4,-62.2}}, color={0,0,127}));
      connect(layFra.port_b, heaCapFraExt.port)
        annotation (Line(points={{-10,70},{-10,100}}, color={191,0,0}));
      connect(heaCapGlaExt.port, layMul.port_b)
        annotation (Line(points={{-10,-12},{-10,0}}, color={191,0,0}));
      connect(radSolData.HDirTil, dirFac.u) annotation (Line(points={{-79.4,-46},
              {-78,-46},{-78,-70},{-70.24,-70}},                    color={0,0,
              127}));
      connect(dirFac.y, shaType.HDirTil) annotation (Line(points={{-61.6,-70},{
              -61.6,-64},{-66,-64},{-66,-44},{-70,-44}}, color={0,0,127}));
      connect(radSolData.HSkyDifTil, skyDifFac.u) annotation (Line(points={{-79.4,
              -48},{-80,-48},{-80,-80},{-80.24,-80}},       color={0,0,127}));
      connect(shaType.HSkyDifTil, skyDifFac.y) annotation (Line(points={{-70,-46},
              {-70,-58},{-72,-58},{-72,-80},{-71.6,-80}},      color={0,0,127}));
        annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-60,-100},{60,100}}),
            graphics={
            Rectangle(
              extent={{-50,-90},{50,100}},
              pattern=LinePattern.None,
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-46,60},{50,24},{50,-50},{-30,-20},{-46,-20},{-46,60}},
              smooth=Smooth.None,
              pattern=LinePattern.None,
              fillColor={255,255,170},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,0}),
            Line(
              points={{-50,60},{-30,60},{-30,80},{50,80}},
              color={175,175,175}),
            Line(
              points={{-50,-20},{-30,-20},{-30,-70},{-30,-70},{52,-70}},
              color={175,175,175}),
            Line(
              points={{-50,60},{-50,66},{-50,100},{50,100}},
              color={175,175,175}),
            Line(
              points={{-50,-20},{-50,-90},{50,-90}},
              color={175,175,175}),
            Line(
              points={{-46,60},{-46,-20}},
              color={0,0,0},
              thickness=0.5,
              smooth=Smooth.None)}),
        Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,
                100}}), graphics={Rectangle(
              extent={{-56,-88},{-90,-64}},
              lineColor={0,255,255},
              fillColor={0,255,255},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This model should be used to model windows or other transparant surfaces.
See <a href=modelica://IDEAS.Buildings.Components.Interfaces.PartialSurface>IDEAS.Buildings.Components.Interfaces.PartialSurface</a> 
for equations, options, parameters, validation and dynamics that are common for all surfaces and windows.
</p>
<h4>Typical use and important parameters</h4>
<p>
Parameter <code>A</code> is the total window surface area, i.e. the
sum of the frame surface area and the glazing surface area.
</p>
<p>
Parameter <code>frac</code> may be used to define the surface
area of the frame as a fraction of <code>A</code>. 
</p>
<p>
Parameter <code>glazing</code>  must be used to define the glass properties.
It contains information about the number of glass layers,
their thickness, thermal properties and emissivity.
</p>
<p>
Optional parameter <code>briType</code> may be used to compute additional line losses
along the edges of the glazing.
</p>
<p>
Optional parameter <code>fraType</code> may be used to define the frame thermal properties.
If <code>fraType = None</code> then the frame is assumed to be perfectly insulating.
</p>
<p>
Optional parameter <code>shaType</code> may be used to define the window shading properties.
</p>
<p>
The parameter <code>n</code> may be used to scale the window to <code>n</code> identical windows.
For example, if a wall has 10 identical windows with identical shading, this parameter
can be used to simulate 10 windows by scaling the model of a single window.
</p>
<h4>Validation</h4>
<p>
To verify the U-value of your glazing system implementation,
see <a href=\"modelica://IDEAS.Buildings.Components.Validations.WindowEN673\">
IDEAS.Buildings.Components.Validations.WindowEN673</a>
</p>
</html>",     revisions="<html>
<ul>
<li>
November 28, 2019, by Ian Beausoleil-Morrison:<br/>
<code>inc</code> and <code>azi</code> of surface now passed as parameters to ExteriorConvection.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/1089\">
#1089</a>
</li>
<li>
December 2, 2019, by Filip Jorissen:<br/>
Split heat capacitor to interior and exterior part 
to avoid non-linear algebraic loops.
<a href=\"https://github.com/open-ideas/IDEAS/issues/1092\">#1092</a>.
</li>
<li>
October 13, 2019, by Filip Jorissen:<br/>
Refactored the parameter definition of <code>inc</code> 
and <code>azi</code> by adding the option to use radio buttons.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/1067\">
#1067</a>
</li>
<li>
September 9, 2019, by Filip Jorissen:<br/>
Added <code>checkCoatings</code> for issue
<a href=\"https://github.com/open-ideas/IDEAS/issues/1038\">#1038</a>.
</li>
<li>
August 10, 2018 by Damien Picard:<br/>
Add scaling to propsBus_a to allow simulation of n windows instead of 1
See <a href=\"https://github.com/open-ideas/IDEAS/issues/888\">
#888</a>.
</li>
<li>
January 21, 2018 by Filip Jorissen:<br/>
Changed implementation such that control input is visible.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/761\">
#761</a>.
</li>
<li>
May 26, 2017 by Filip Jorissen:<br/>
Revised implementation for renamed
ports <code>HDirTil</code> etc.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/735\">
#735</a>.
</li>
<li>
March 6, 2017, by Filip Jorissen:<br/>
Added option for using 'normal' dynamics for the window glazing.
Removed the option for having a combined state for 
window and frame this this is non-physical.
This is for 
<a href=https://github.com/open-ideas/IDEAS/issues/678>#678</a>.
</li>
<li>
January 10, 2017, by Filip Jorissen:<br/>
Removed declaration of 
<code>A</code> since this is now declared in 
<a href=modelica://IDEAS.Buildings.Components.Interfaces.PartialSurface>
IDEAS.Buildings.Components.Interfaces.PartialSurface</a>.
This is for 
<a href=https://github.com/open-ideas/IDEAS/issues/609>#609</a>.
</li>
<li>
January 10, 2017 by Filip Jorissen:<br/>
Set <code>linExtRad = sim.linExtRadWin</code>.
See <a href=https://github.com/open-ideas/IDEAS/issues/615>#615</a>.
</li>
<li>
December 19, 2016, by Filip Jorissen:<br/>
Added solar irradiation on window frame.
</li>
<li>
December 19, 2016, by Filip Jorissen:<br/>
Removed briType, which had default value LineLoss.
briType is now part of the Frame model and has default
value None.
</li>
<li>
February 10, 2016, by Filip Jorissen and Damien Picard:<br/>
Revised implementation: cleaned up connections and partials.
</li>
<li>
December 17, 2015, Filip Jorissen:<br/>
Added thermal connection between frame and glazing state. 
This is required for decoupling steady state thermal dynamics
without adding a second state for the window.
</li>
<li>
July 14, 2015, Filip Jorissen:<br/>
Removed second shading device since a new partial was created
for handling this.
</li>
<li>
June 14, 2015, Filip Jorissen:<br/>
Adjusted implementation for computing conservation of energy.
</li>
<li>
February 10, 2015 by Filip Jorissen:<br/>
Adjusted implementation for grouping of solar calculations.
</li>
</ul>
</html>"));
    end Window;

    model DirFac "Multiplication factor for external shading - Dir"
      parameter Boolean shDir = false "Set true to multiply with external shading factor";
      Real fac;

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/3/3c/3cforBEMB1.mat",
        columns={3})
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
    equation
      fac = if shDir then combiTimeTable.y[1] else 1;
      y=u*fac;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=31536000));
    end DirFac;

    model SkyDifFac
      "Multiplication factor for external shading - Sky Diff"
      parameter Boolean shDif = false "Set true to multiply with external shading factor";
      Real fac;

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/3/3c/3cforBEMB1.mat",
        columns={2})
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
    equation
      fac = if shDif then combiTimeTable.y[1] else 1;
      y=u*fac;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end SkyDifFac;

    record ExternalWall0 "External Wall original (based on MSc table)"
      extends IDEAS.Buildings.Data.Interfaces.Construction(
        mats={IDEAS.Buildings.Data.Materials.BrickHe(k=1.4, c=1000, rho=1900, d=0.09,epsLw=0.93),
              IDEAS.Buildings.Data.Materials.Air(d=0.03),
              IDEAS.Buildings.Data.Insulation.Pur(k=0.028,c=1400, rho=35, d=0.08),
              IDEAS.Buildings.Data.Materials.BrickMe(k=0.51,c=1000, rho=1400, d=0.14),
              IDEAS.Buildings.Data.Materials.Gypsum(k=0.52,c=1000, rho=1300, d=0.01,epsSw=0.17)});

      annotation (Documentation(info="<html>
<p>
Example implementation of a cavity wall.
</p>
<ul>
<li>
November 14, 2016, by Filip Jorissen:<br/>
Revised implementation: removed insulationType.
</li>
</ul>
</html>"));
    end ExternalWall0;
  end ConstructionComponents;

  package OfficeBEM
    model OfficeRev2
      extends Modelica.Icons.Example;
      package Medium = IDEAS.Media.Air "Air medium";

      parameter Modelica.SIunits.Length l = 4 "Zone length";
      parameter Modelica.SIunits.Length w = 2.7 "Zone width";
      parameter Modelica.SIunits.Length h = 2.8 "Zone height";
      parameter Modelica.SIunits.Length h2 = 0.5 "Plenum height";
      parameter Modelica.SIunits.Length wwin = 2 "Window width";
      parameter Modelica.SIunits.Length hwin = 1.6 "Window height";
      parameter Real ventRate = (36*1.2)/3600 "Ventilation Rate in kg/s";
      Modelica.SIunits.Power QHtotal;
      Modelica.SIunits.Power QCtotal;

         model OccSched "Simple occupancy schedule"
        extends
          IDEAS.Buildings.Components.Occupants.BaseClasses.PartialOccupants(      final useInput=false);

        parameter Real k "Number of occupants";
        IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
          annotation (Placement(transformation(extent={{-20,20},{0,40}})));
        Modelica.Blocks.Sources.RealExpression occ(y=if calTim.weekDay < 6 and (
              calTim.hour > 8 and calTim.hour < 18) then k else 0)
          "Number of occupants present"
          annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
         equation
        connect(occ.y, nOcc)
          annotation (Line(points={{1,0},{120,0}}, color={0,0,127}));
         end OccSched;

      //SimInfoManager must be 'inner' at the top level
      inner IDEAS.BoundaryConditions.SimInfoManager sim
        annotation (Placement(transformation(extent={{-200,78},{-180,98}})));
      IDEAS.Buildings.Components.Zone OfficeCell(
        redeclare package Medium = Medium,
        nSurf=7,
        V=l*h*w,
        hZone=h,
        n50=2,
        redeclare OccSched occNum(k=1)) "Zone model"
        annotation (Placement(transformation(extent={{-20,0},{0,20}})));
      IDEAS.Buildings.Components.OuterWall outerWall3(
        redeclare ConstructionComponents.ExternalWall0 constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.W,
        A=w*h - window.A)
        "Outer wall model"
         annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=90,
            origin={-10,-30})));
      IDEAS.Buildings.Components.Window window(
        inc=IDEAS.Types.Tilt.Wall,
        A=wwin*hwin,
        azi=IDEAS.Types.Azimuth.W,
        redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
        redeclare IDEAS.Buildings.Components.Shading.Screen shaType)
        "Window model" annotation (Placement(transformation(extent={{-6,10},{6,
                -10}},
            rotation=90,
            origin={-36,-30})));

      IDEAS.Buildings.Components.InternalWall floor(
        redeclare ConstructionComponents.SuspCeiling constructionType,
        inc=IDEAS.Types.Tilt.Ceiling,
        azi=IDEAS.Types.Azimuth.S,
        A=l*w)
        "Floor modelled using internal wall with both the ceiling and roof side connected to the zone"
        annotation (Placement(transformation(
            extent={{6,-10},{-6,10}},
            rotation=90,
            origin={-110,10})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.E,
        A=h*w,
        use_T_in=true) annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=-90,
            origin={-10,58})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall1(
        redeclare ConstructionComponents.InternalWall constructionType,
        incOpt=4,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.N,
        A=h*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-76,18},{-64,38}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall2(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.S,
        A=h*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-74,-12},{-62,8}})));
      IDEAS.Templates.Heating.IdealRadiatorHeating Heating(
        nZones=1,
        Q_design=zeros(Heating.nZones),
        QNom=fill(1000, Heating.nZones),
        fraRad=0.7)
        annotation (Placement(transformation(extent={{122,76},{162,96}})));
      BEM.Controls.HeatingSetT HeatingSetT(multi=273.15 + 22)
        annotation (Placement(transformation(extent={{172,58},{160,70}})));
      IDEAS.Templates.Heating.IdealRadiatorHeating Cooling(
        nZones=1,
        Q_design=zeros(Cooling.nZones),
        dTHys=2,
        QNom=fill(-1000, Cooling.nZones),
        fraRad=1)
        annotation (Placement(transformation(extent={{136,28},{176,8}})));
      BEM.Controls.CoolingSetT CoolingSetT(multi=273.15 + 23)
        annotation (Placement(transformation(extent={{104,12},{114,22}})));

      IDEAS.Fluid.Sources.MassFlowSource_T boundary(
        redeclare package Medium = Medium,
        use_m_flow_in=true,
        T=290.15,
        nPorts=1) annotation (Placement(transformation(extent={{78,-64},{58,-44}})));
      IDEAS.Fluid.Sources.OutsideAir outsideAir(redeclare package Medium = Medium,
          nPorts=1)
        annotation (Placement(transformation(extent={{78,-34},{58,-14}})));
      Controls.IntGain intGain(Qon=13*OfficeCell.A, Qoff=4*OfficeCell.A)
        annotation (Placement(transformation(extent={{-70,-90},{-58,-78}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlowCon
        annotation (Placement(transformation(extent={{-32,-84},{-12,-64}})));
      Modelica.Blocks.Math.Gain gain(k=0.5)
        annotation (Placement(transformation(extent={{-52,-88},{-44,-80}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlowRad
        annotation (Placement(transformation(extent={{-32,-102},{-12,-82}})));
      Controls.VentQ ventQ(C=1000, ventT=boundary.T) annotation (Placement(
            transformation(
            extent={{-6,6},{6,-6}},
            rotation=180,
            origin={90,-68})));
      Controls.VentIPd ventIPd(ven=ventRate)
        annotation (Placement(transformation(extent={{114,-54},{104,-44}})));
      Modelica.Blocks.Continuous.Integrator JCooling(k=1/(3600000*OfficeCell.A),
          u=QCtotal)
        annotation (Placement(transformation(extent={{208,70},{228,90}})));
      Modelica.Blocks.Continuous.Integrator JHeating(k=1/(3600000*OfficeCell.A),
          u=QHtotal)
        annotation (Placement(transformation(extent={{208,18},{228,38}})));
      IDEAS.Buildings.Components.Zone plenum(
        redeclare package Medium = Medium,
        nSurf=6,
        V=l*h2*w,
        hZone=h2) "Zone model"
        annotation (Placement(transformation(extent={{-152,-8},{-132,12}})));
      IDEAS.Buildings.Components.OuterWall outerWall1(
        redeclare ConstructionComponents.ExternalWall0 constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.W,
        A=w*h2)
        "Outer wall model"
         annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=90,
            origin={-148,-20})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall3(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.S,
        A=h2*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-180,-18},{-168,2}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall4(
        redeclare ConstructionComponents.InternalWall constructionType,
        incOpt=4,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.N,
        A=h2*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-180,12},{-168,32}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall5(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=IDEAS.Types.Azimuth.E,
        A=h2*w,
        use_T_in=true) annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=-90,
            origin={-142,36})));
      IDEAS.Buildings.Components.InternalWall floor1(
        redeclare ConstructionComponents.ElevFloor1 constructionType,
        inc=IDEAS.Types.Tilt.Floor,
        azi=IDEAS.Types.Azimuth.S,
        A=l*w)
        "Floor modelled using internal wall with both the ceiling and roof side connected to the zone"
        annotation (Placement(transformation(
            extent={{6,-10},{-6,10}},
            rotation=90,
            origin={-106,-10})));
      Controls.ScreenControl1 screenControl1_1(IrrTot=window.shaType.HDirTil+window.shaType.HSkyDifTil+window.shaType.HGroDifTil)
        annotation (Placement(transformation(extent={{-66,-40},{-54,-28}})));
    equation
      ventQ.ambT = sim.Te;
      QHtotal = ventQ.QvenHeatg + Heating.QHeaSys;
      QCtotal = ventQ.QvenCoolg + Cooling.QCooTotal;

      connect(outerWall3.propsBus_a, OfficeCell.propsBus[4]) annotation (Line(
          points={{-12,-25},{-12,-6},{-20,-6},{-20,14}},
          color={255,204,51},
          thickness=0.5));
      connect(window.propsBus_a, OfficeCell.propsBus[5]) annotation (Line(
          points={{-34,-25},{-34,14},{-20,14},{-20,13.4286}},
          color={255,204,51},
          thickness=0.5));
      connect(floor.propsBus_a, OfficeCell.propsBus[6]) annotation (Line(
          points={{-112,5},{-112,4},{-20,4},{-20,12.8571}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall.propsBus_a, OfficeCell.propsBus[2]) annotation (Line(
          points={{-8,53},{-24,53},{-24,15.1429},{-20,15.1429}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall1.propsBus_a, OfficeCell.propsBus[1]) annotation (
          Line(
          points={{-65,30},{-38,30},{-38,15.7143},{-20,15.7143}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall2.propsBus_a, OfficeCell.propsBus[3]) annotation (
          Line(
          points={{-63,0},{-40,0},{-40,14.5714},{-20,14.5714}},
          color={255,204,51},
          thickness=0.5));
      connect(OfficeCell.TSensor, Heating.TSensor[1]) annotation (Line(points={
              {1,12},{7.5,12},{7.5,80},{121.6,80}}, color={0,0,127}));
      connect(OfficeCell.gainCon, Heating.heatPortCon[1]) annotation (Line(
            points={{0,7},{4,7},{4,88},{122,88}}, color={191,0,0}));
      connect(OfficeCell.gainRad, Heating.heatPortRad[1]) annotation (Line(
            points={{0,4},{6,4},{6,84},{122,84}}, color={191,0,0}));
      connect(HeatingSetT.y, Heating.TSet[1]) annotation (Line(points={{159.4,
              64},{142,64},{142,75.8}},
                               color={0,0,127}));
      connect(OfficeCell.gainRad, Cooling.heatPortRad[1]) annotation (Line(
            points={{0,4},{118,4},{118,20},{136,20}}, color={191,0,0}));
      connect(Cooling.heatPortCon[1], OfficeCell.gainCon) annotation (Line(
            points={{136,16},{120,16},{120,7},{0,7}}, color={191,0,0}));
      connect(Cooling.TSet[1], OfficeCell.TSensor) annotation (Line(points={{
              156,28.2},{84,28.2},{84,28},{10,28},{10,12},{1,12}}, color={0,0,
              127}));
      connect(CoolingSetT.y, Cooling.TSensor[1]) annotation (Line(points={{114.5,17},
              {114.5,24.5},{135.6,24.5},{135.6,24}}, color={0,0,127}));

      connect(outsideAir.ports[1], OfficeCell.port_a) annotation (Line(points={
              {58,-24},{26,-24},{26,20},{-8,20}}, color={0,127,255}));
      connect(intGain.y,gain. u) annotation (Line(points={{-57.4,-84},{-52.8,-84}},
                                     color={0,0,127}));
      connect(prescribedHeatFlowCon.Q_flow,gain. y) annotation (Line(points={{-32,-74},
              {-40,-74},{-40,-84},{-43.6,-84}},      color={0,0,127}));
      connect(gain.y,prescribedHeatFlowRad. Q_flow) annotation (Line(points={{-43.6,
              -84},{-41.65,-84},{-41.65,-92},{-32,-92}},       color={0,0,127}));
      connect(prescribedHeatFlowCon.port, OfficeCell.gainCon) annotation (Line(
            points={{-12,-74},{2,-74},{2,7},{0,7}}, color={191,0,0}));
      connect(prescribedHeatFlowRad.port, OfficeCell.gainRad) annotation (Line(
            points={{-12,-92},{4,-92},{4,4},{0,4}}, color={191,0,0}));
      connect(boundary.ports[1], OfficeCell.port_b) annotation (Line(points={{
              58,-54},{24,-54},{24,20},{-12,20}}, color={0,127,255}));
      connect(ventIPd.y, boundary.m_flow_in) annotation (Line(points={{103.5,-49},{92.5,
              -49},{92.5,-46},{80,-46}}, color={0,0,127}));
      connect(boundaryWall1.T, OfficeCell.TSensor) annotation (Line(points={{-81,
              30},{-40,30},{-40,12},{1,12}}, color={0,0,127}));
      connect(boundaryWall2.T, OfficeCell.TSensor) annotation (Line(points={{-79,
              0},{-38,0},{-38,12},{1,12}}, color={0,0,127}));
      connect(boundaryWall.T, OfficeCell.TSensor) annotation (Line(points={{-8,
              69},{-4,69},{-4,12},{1,12}}, color={0,0,127}));
      connect(ventIPd.y, ventQ.ventCtrl) annotation (Line(points={{103.5,-49},{89.75,
              -49},{89.75,-61.76},{90,-61.76}}, color={0,0,127}));
      connect(floor.propsBus_b, plenum.propsBus[1]) annotation (Line(
          points={{-112,15},{-124,15},{-124,7.66667},{-152,7.66667}},
          color={255,204,51},
          thickness=0.5));
      connect(outerWall1.propsBus_a, plenum.propsBus[2]) annotation (Line(
          points={{-150,-15},{-150,-2.5},{-152,-2.5},{-152,7}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall3.propsBus_a, plenum.propsBus[3]) annotation (Line(
          points={{-169,-6},{-160,-6},{-160,6.33333},{-152,6.33333}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall3.T, plenum.TSensor) annotation (Line(points={{-185,-6},
              {-156,-6},{-156,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall4.T, plenum.TSensor) annotation (Line(points={{-185,
              24},{-154,24},{-154,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall4.propsBus_a, plenum.propsBus[4]) annotation (Line(
          points={{-169,24},{-169,25},{-152,25},{-152,5.66667}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall5.T, plenum.TSensor) annotation (Line(points={{-140,
              47},{-132,47},{-132,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall5.propsBus_a, plenum.propsBus[5]) annotation (Line(
          points={{-140,31},{-144,31},{-144,5},{-152,5}},
          color={255,204,51},
          thickness=0.5));
      connect(floor1.propsBus_a, OfficeCell.propsBus[7]) annotation (Line(
          points={{-108,-15},{-108,-18},{-90,-18},{-90,10},{-56,10},{-56,
              12.2857},{-20,12.2857}},
          color={255,204,51},
          thickness=0.5));
      connect(floor1.propsBus_b, plenum.propsBus[6]) annotation (Line(
          points={{-108,-5},{-108,-1.5},{-152,-1.5},{-152,4.33333}},
          color={255,204,51},
          thickness=0.5));
      connect(window.Ctrl, screenControl1_1.y)
        annotation (Line(points={{-46,-34},{-53.52,-34}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}})),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,
                100}})),
        experiment(
          StopTime=31536000,
          Interval=600,
          Tolerance=1e-06,
          __Dymola_Algorithm="Lsodar"),
        __Dymola_Commands(file=
              "Resources/Scripts/Dymola/Examples/Tutorial/Example1.mos"
            "Simulate and plot"),
        Documentation(info="<html>
<p>
This first example file instantiates a simple building model.
</p>
</html>",     revisions="<html>
<ul>
<li>
September 18, 2019 by Filip Jorissen:<br/>
First implementation for the IDEAS crash course.
</li>
</ul>
</html>"));
    end OfficeRev2;
  end OfficeBEM;

  package Controls
    model ScreenControl1 "Irr based screen control"
      parameter Real setIrr(unit="W/m2") = 100 "Solar Irradiation at which screen is activated";
      input Real IrrTot(unit="W/m2");
      Real shaFac;

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{98,-10},{118,10}})));
    equation
      shaFac = FNShaFacIrr(setIrr, IrrTot);
      y=shaFac;
        annotation (Line(points={{11,0},{108,0}}, color={0,0,127}));
    end ScreenControl1;

    model VentIPd "Ventilation rate input 7-19"

      parameter Real ven = 0.1 "Ventilation flow rate";

    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 6 and calTim.hour < 19) then ven else 0)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end VentIPd;

    model VentQ "Ventilation energy demand model"

      parameter Real C = 1000 "Heat capacity of air, in J/kg.K";
      parameter Modelica.SIunits.Temperature ventT = 10 "Ventilation temperature";
      Modelica.SIunits.Power QvenHeatg;
      Modelica.SIunits.Power QvenCoolg;

      Modelica.Blocks.Interfaces.RealInput ambT
        annotation (Placement(transformation(extent={{-120,-100},{-80,-60}})));
      Modelica.Blocks.Interfaces.RealInput ventCtrl annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=-90,
            origin={0,104})));
    protected
      Modelica.SIunits.Temperature delT=ventT - ambT;

    equation

      QvenHeatg = ventCtrl*max(0, C*delT);
      QvenCoolg = ventCtrl*max(0, C*(-delT));

    end VentQ;

    model VentIPc "Ventilation rate input 7-19"

      parameter Real ven = 0.1 "Ventilation flow rate";

    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 6 and calTim.hour < 19) then ven else 0)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end VentIPc;

    model ShadingControl "Window blinds shading input"

      parameter Real shafac=1 "Window screen control input";
      IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
        annotation (Placement(transformation(extent={{-20,20},{0,40}})));
      Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
              calTim.hour > 7 and calTim.hour < 18) then shafac else 0)
      annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y) annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
      annotation (experiment(
          StartTime=10000000,
          StopTime=15000000,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Lsodar"));
    end ShadingControl;

    model Cooling

      parameter Real dTHys(min=0)=2;
      parameter Real QNom=1000;
      parameter Real fraRad=0.3;

      Modelica.Blocks.Interfaces.RealInput TSensor
        annotation (Placement(transformation(extent={{-132,-108},{-92,-68}})));
      Modelica.Blocks.Interfaces.RealInput TSet annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={80,-106})));
      Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={86,-42})));
      Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=0, uHigh=dTHys)
        annotation (Placement(transformation(extent={{80,-20},{60,0}})));
      Modelica.Blocks.Math.BooleanToReal booleanToReal
        annotation (Placement(transformation(extent={{52,-20},{32,0}})));
      Modelica.Blocks.Math.Gain gain(k=QNom)
        annotation (Placement(transformation(extent={{24,-20},{4,0}})));
      Modelica.Blocks.Math.Gain gain1(k=1 - fraRad)
        annotation (Placement(transformation(extent={{-4,-46},{-24,-26}})));
      Modelica.Blocks.Math.Gain gain2(k=fraRad)
        annotation (Placement(transformation(extent={{-4,4},{-24,24}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlow
        annotation (Placement(transformation(extent={{-32,-46},{-52,-26}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlow1
        annotation (Placement(transformation(extent={{-32,4},{-52,24}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a HeatPortCon
        annotation (Placement(transformation(extent={{-110,6},{-90,26}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a HeatPortRad
        annotation (Placement(transformation(extent={{-112,-46},{-92,-26}})));
    equation
      connect(TSet, add.u2) annotation (Line(points={{80,-106},{80,-81},{92,-81},{92,
              -54}}, color={0,0,127}));
      connect(TSensor, add.u1) annotation (Line(points={{-112,-88},{60,-88},{60,-54},
              {80,-54}}, color={0,0,127}));
      connect(add.y, hysteresis.u)
        annotation (Line(points={{86,-31},{86,-10},{82,-10}}, color={0,0,127}));
      connect(hysteresis.y, booleanToReal.u)
        annotation (Line(points={{59,-10},{54,-10}}, color={255,0,255}));
      connect(booleanToReal.y, gain.u)
        annotation (Line(points={{31,-10},{26,-10}}, color={0,0,127}));
      connect(gain2.u, gain.y) annotation (Line(points={{-2,14},{2,14},{2,-10},
              {3,-10}}, color={0,0,127}));
      connect(gain.y, gain1.u) annotation (Line(points={{3,-10},{2,-10},{2,-36},
              {-2,-36}}, color={0,0,127}));
      connect(gain1.y, prescribedHeatFlow.Q_flow)
        annotation (Line(points={{-25,-36},{-32,-36}}, color={0,0,127}));
      connect(gain2.y, prescribedHeatFlow1.Q_flow)
        annotation (Line(points={{-25,14},{-32,14}}, color={0,0,127}));
      connect(prescribedHeatFlow1.port, HeatPortCon) annotation (Line(points={{
              -52,14},{-76,14},{-76,16},{-100,16}}, color={191,0,0}));
      connect(prescribedHeatFlow.port, HeatPortRad) annotation (Line(points={{
              -52,-36},{-74,-36},{-74,-36},{-102,-36}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Cooling;

    model VentIPb "Ventilation rate input 18-19"

      parameter Real ven = 0.1 "Ventilation flow rate";

    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 17 and calTim.hour < 19) then ven else 0)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end VentIPb;

    model VentIPa "Ventilation rate input 7-8"

      parameter Real ven = 0.1 "Ventilation flow rate";

    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 6 and calTim.hour < 8) then ven else 0)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end VentIPa;

    model VentIP "Ventilation rate input"

      parameter Real ven = 0.1 "Ventilation flow rate";

    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 8 and calTim.hour < 18) then ven else 0)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end VentIP;

    model CoolingSetT "Heating, Cooling & Ventilation schedule"

      parameter Real multi=273.15 "Temperature during office hours";
      IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
        annotation (Placement(transformation(extent={{-20,20},{0,40}})));
      Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
              calTim.hour > 7 and calTim.hour < 18) then multi else 1000)
      annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y) annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
      annotation (experiment(
          StartTime=10000000,
          StopTime=15000000,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Lsodar"));
    end CoolingSetT;

    model HeatingSetT "Heating, Cooling & Ventilation schedule"

      parameter Real multi=273.15 "Temperature during office hours";
      IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
        annotation (Placement(transformation(extent={{-20,20},{0,40}})));
      Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
              calTim.hour > 7 and calTim.hour < 18) then multi else 0)
      annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y) annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
      annotation (experiment(
          StopTime=604800,
          __Dymola_NumberOfIntervals=5000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Lsodar"));
    end HeatingSetT;

    model IntGain "Appliance & Lighting heat gains input"

      parameter Real Qon = 10 "Thermal gain during office hours";
      parameter Real Qoff = 5 "Thermal gains during non-office hours";
    IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Modelica.Blocks.Sources.RealExpression op(y=if calTim.weekDay < 6 and (
            calTim.hour > 7 and calTim.hour < 18) then Qon else Qoff)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(op.y, y)
        annotation (Line(points={{1,0},{110,0}}, color={0,0,127}));
    annotation (experiment(
        StartTime=10000000,
        StopTime=15000000,
        __Dymola_NumberOfIntervals=5000,
        Tolerance=1e-06,
        __Dymola_Algorithm="Lsodar"));
    end IntGain;

    function FNShaFacIrr "Shading Factor function based on Irradiation"
        input Real Irr0;
        input Real IrrW;
        output Real sf;

    algorithm

      if IrrW>Irr0 then
        sf :=1;
      else sf :=0;
      end if;

    end FNShaFacIrr;

    model ScreenControl2 "zoneT & Irr based screen control"
      parameter Modelica.SIunits.Temperature setT = 283.15 "Temperature at which screen is activated";
      input Modelica.SIunits.Temperature zoneT;
      Real shaFac1;
      parameter Real setIrr(unit="W/m2") = 100 "Solar Irradiation at which screen is activated";
      input Real IrrTot(unit="W/m2");
      Real shaFac2;
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{98,-10},{118,10}})));
    equation
      shaFac1 =FNShaFacT(setT, zoneT);
      shaFac2 = FNShaFacIrr(setIrr, IrrTot);
      y = max(shaFac1,shaFac2);
    end ScreenControl2;

    function FNShaFacT "Shading Factor function based on temperature"
        input Real T0;
        input Real Tz;
        output Real sf;

    algorithm

      if Tz>T0 then
        sf :=1;
      else sf :=0;
      end if;

    end FNShaFacT;
  end Controls;

  package OfficeZave250520 "BEM of office complex in Zaventem"
    model OffZave1
      extends Modelica.Icons.Example;
      package Medium = IDEAS.Media.Air "Air medium";

      parameter Modelica.SIunits.Length l = 4 "Zone length";
      parameter Modelica.SIunits.Length w = 2.7 "Zone width";
      parameter Modelica.SIunits.Length h = 2.8 "Zone height";
      parameter Modelica.SIunits.Length h2 = 0.5 "Plenum height";
      parameter Modelica.SIunits.Length wwin = w "Window width";
      parameter Modelica.SIunits.Length hwin = h/2 "Window height";
      parameter Real ventRate = (36*1.2)/3600 "Ventilation Rate in kg/s";

      Modelica.Blocks.Continuous.Integrator JCooling(k=1/(3600000*OfficeCell.A),
          u=QCtotal)
        annotation (Placement(transformation(extent={{208,70},{228,90}})));
      Modelica.Blocks.Continuous.Integrator JHeating(k=1/(3600000*OfficeCell.A),
          u=QHtotal)
        annotation (Placement(transformation(extent={{208,18},{228,38}})));

    //protected
      Modelica.SIunits.Power QHtotal;
      Modelica.SIunits.Power QCtotal;

         model OccSched "Simple occupancy schedule"
        extends IDEAS.Buildings.Components.Occupants.BaseClasses.PartialOccupants(final useInput=false);

        parameter Real k "Number of occupants";
        IDEAS.Utilities.Time.CalendarTime calTim(zerTim=IDEAS.Utilities.Time.Types.ZeroTime.NY2019)
          annotation (Placement(transformation(extent={{-20,20},{0,40}})));
        Modelica.Blocks.Sources.RealExpression occ(y=if calTim.weekDay < 6 and (
              calTim.hour > 8 and calTim.hour < 18) then k else 0)
          "Number of occupants present"
          annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
         equation
        connect(occ.y, nOcc)
          annotation (Line(points={{1,0},{120,0}}, color={0,0,127}));
         end OccSched;

      //SimInfoManager must be 'inner' at the top level
      inner IDEAS.BoundaryConditions.SimInfoManager sim
        annotation (Placement(transformation(extent={{-200,78},{-180,98}})));
      IDEAS.Buildings.Components.Zone OfficeCell(
        redeclare package Medium = Medium,
        nSurf=7,
        V=l*h*w,
        hZone=h,
        n50=2,
        redeclare OccSched occNum(k=1)) "Zone model"
        annotation (Placement(transformation(extent={{-20,0},{0,20}})));
      IDEAS.Buildings.Components.OuterWall outerWall3(
        redeclare ConstructionComponents.ExternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=0.75049157835756,
        A=w*h - window.A)
        "Outer wall model"
         annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=90,
            origin={-10,-30})));

      IDEAS.Buildings.Components.InternalWall floor(
        redeclare ConstructionComponents.SuspCeiling constructionType,
        inc=IDEAS.Types.Tilt.Ceiling,
        azi=IDEAS.Types.Azimuth.S,
        A=l*w)
        "Floor modelled using internal wall with both the ceiling and roof side connected to the zone"
        annotation (Placement(transformation(
            extent={{6,-10},{-6,10}},
            rotation=90,
            origin={-110,10})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=-2.3911010752322,
        A=h*w,
        use_T_in=true) annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=-90,
            origin={-10,58})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall1(
        redeclare ConstructionComponents.InternalWall constructionType,
        incOpt=4,
        inc=IDEAS.Types.Tilt.Wall,
        azi=2.3212879051525,
        A=h*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-76,18},{-64,38}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall2(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=-0.82030474843733,
        A=h*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-74,-12},{-62,8}})));
      IDEAS.Templates.Heating.IdealRadiatorHeating Heating(
        nZones=1,
        Q_design=zeros(Heating.nZones),
        QNom=fill(1000, Heating.nZones),
        fraRad=0.7)
        annotation (Placement(transformation(extent={{122,76},{162,96}})));
      BEM.Controls.HeatingSetT HeatingSetT(multi=273.15 + 22)
        annotation (Placement(transformation(extent={{172,58},{160,70}})));
      IDEAS.Templates.Heating.IdealRadiatorHeating Cooling(
        nZones=1,
        Q_design=zeros(Cooling.nZones),
        dTHys=2,
        QNom=fill(-1000, Cooling.nZones),
        fraRad=1)
        annotation (Placement(transformation(extent={{136,28},{176,8}})));
      BEM.Controls.CoolingSetT CoolingSetT(multi=273.15 + 23)
        annotation (Placement(transformation(extent={{104,12},{114,22}})));

      IDEAS.Fluid.Sources.MassFlowSource_T boundary(
        redeclare package Medium = Medium,
        use_m_flow_in=true,
        T=290.15,
        nPorts=1) annotation (Placement(transformation(extent={{78,-64},{58,-44}})));
      IDEAS.Fluid.Sources.OutsideAir outsideAir(redeclare package Medium = Medium,
          nPorts=1)
        annotation (Placement(transformation(extent={{78,-34},{58,-14}})));
      Controls.IntGain intGain(Qon=13*OfficeCell.A, Qoff=4*OfficeCell.A)
        annotation (Placement(transformation(extent={{-70,-90},{-58,-78}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlowCon
        annotation (Placement(transformation(extent={{-32,-84},{-12,-64}})));
      Modelica.Blocks.Math.Gain gain(k=0.5)
        annotation (Placement(transformation(extent={{-52,-88},{-44,-80}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
        prescribedHeatFlowRad
        annotation (Placement(transformation(extent={{-32,-102},{-12,-82}})));
      Controls.VentQ ventQ(C=1000, ventT=boundary.T) annotation (Placement(
            transformation(
            extent={{-6,6},{6,-6}},
            rotation=180,
            origin={90,-68})));
      Controls.VentIPd ventIPd(ven=ventRate)
        annotation (Placement(transformation(extent={{114,-54},{104,-44}})));

      IDEAS.Buildings.Components.OuterWall outerWall1(
        redeclare ConstructionComponents.ExternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=outerWall3.azi,
        A=w*h2)
        "Outer wall model"
         annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=90,
            origin={-148,-20})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall3(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=boundaryWall2.azi,
        A=h2*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-180,-18},{-168,2}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall4(
        redeclare ConstructionComponents.InternalWall constructionType,
        incOpt=4,
        inc=IDEAS.Types.Tilt.Wall,
        azi=boundaryWall1.azi,
        A=h2*l,
        use_T_in=true)
        annotation (Placement(transformation(extent={{-180,12},{-168,32}})));
      IDEAS.Buildings.Components.BoundaryWall boundaryWall5(
        redeclare ConstructionComponents.InternalWall constructionType,
        inc=IDEAS.Types.Tilt.Wall,
        azi=boundaryWall.azi,
        A=h2*w,
        use_T_in=true) annotation (Placement(transformation(
            extent={{-6,-10},{6,10}},
            rotation=-90,
            origin={-142,36})));
      IDEAS.Buildings.Components.InternalWall floor1(
        redeclare ConstructionComponents.ElevFloor1 constructionType,
        inc=IDEAS.Types.Tilt.Floor,
        azi=IDEAS.Types.Azimuth.S,
        A=l*w)
        "Floor modelled using internal wall with both the ceiling and roof side connected to the zone"
        annotation (Placement(transformation(
            extent={{6,-10},{-6,10}},
            rotation=90,
            origin={-106,-10})));
      IDEAS.Buildings.Components.Zone plenum(
        redeclare package Medium = Medium,
        nSurf=6,
        V=l*h2*w,
        hZone=h2) "Zone model"
        annotation (Placement(transformation(extent={{-152,-8},{-132,12}})));
      ConstructionComponents.Window window(
        redeclare IDEAS.Buildings.Data.Glazing.Ins2Ar glazing,
        inc=IDEAS.Types.Tilt.Wall,
        azi=outerWall3.azi,
        A=wwin*hwin,
        redeclare IDEAS.Buildings.Components.Shading.Screen shaType)
        annotation (Placement(transformation(
            extent={{-6,10},{6,-10}},
            rotation=90,
            origin={-36,-30})));
      Controls.ScreenControl1 screenControl1_1(IrrTot=window.shaType.HDirTil + window.shaType.HSkyDifTil + window.shaType.HGroDifTil)
        annotation (Placement(transformation(extent={{-76,-42},{-60,-26}})));
    equation
      ventQ.ambT = sim.Te;
      QHtotal = ventQ.QvenHeatg + Heating.QHeaSys;
      QCtotal = ventQ.QvenCoolg + Cooling.QCooTotal;

      connect(outerWall3.propsBus_a, OfficeCell.propsBus[4]) annotation (Line(
          points={{-12,-25},{-12,-6},{-20,-6},{-20,14}},
          color={255,204,51},
          thickness=0.5));
      connect(floor.propsBus_a, OfficeCell.propsBus[6]) annotation (Line(
          points={{-112,5},{-112,4},{-20,4},{-20,12.8571}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall.propsBus_a, OfficeCell.propsBus[2]) annotation (Line(
          points={{-8,53},{-24,53},{-24,15.1429},{-20,15.1429}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall1.propsBus_a, OfficeCell.propsBus[1]) annotation (
          Line(
          points={{-65,30},{-38,30},{-38,15.7143},{-20,15.7143}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall2.propsBus_a, OfficeCell.propsBus[3]) annotation (
          Line(
          points={{-63,0},{-40,0},{-40,14.5714},{-20,14.5714}},
          color={255,204,51},
          thickness=0.5));
      connect(OfficeCell.TSensor, Heating.TSensor[1]) annotation (Line(points={
              {1,12},{7.5,12},{7.5,80},{121.6,80}}, color={0,0,127}));
      connect(OfficeCell.gainCon, Heating.heatPortCon[1]) annotation (Line(
            points={{0,7},{4,7},{4,88},{122,88}}, color={191,0,0}));
      connect(OfficeCell.gainRad, Heating.heatPortRad[1]) annotation (Line(
            points={{0,4},{6,4},{6,84},{122,84}}, color={191,0,0}));
      connect(HeatingSetT.y, Heating.TSet[1]) annotation (Line(points={{159.4,
              64},{142,64},{142,75.8}},
                               color={0,0,127}));
      connect(OfficeCell.gainRad, Cooling.heatPortRad[1]) annotation (Line(
            points={{0,4},{118,4},{118,20},{136,20}}, color={191,0,0}));
      connect(Cooling.heatPortCon[1], OfficeCell.gainCon) annotation (Line(
            points={{136,16},{120,16},{120,7},{0,7}}, color={191,0,0}));
      connect(Cooling.TSet[1], OfficeCell.TSensor) annotation (Line(points={{
              156,28.2},{84,28.2},{84,28},{10,28},{10,12},{1,12}}, color={0,0,
              127}));
      connect(CoolingSetT.y, Cooling.TSensor[1]) annotation (Line(points={{114.5,17},
              {114.5,24.5},{135.6,24.5},{135.6,24}}, color={0,0,127}));

      connect(outsideAir.ports[1], OfficeCell.port_a) annotation (Line(points={
              {58,-24},{26,-24},{26,20},{-8,20}}, color={0,127,255}));
      connect(intGain.y,gain. u) annotation (Line(points={{-57.4,-84},{-52.8,-84}},
                                     color={0,0,127}));
      connect(prescribedHeatFlowCon.Q_flow,gain. y) annotation (Line(points={{-32,-74},
              {-40,-74},{-40,-84},{-43.6,-84}},      color={0,0,127}));
      connect(gain.y,prescribedHeatFlowRad. Q_flow) annotation (Line(points={{-43.6,
              -84},{-41.65,-84},{-41.65,-92},{-32,-92}},       color={0,0,127}));
      connect(prescribedHeatFlowCon.port, OfficeCell.gainCon) annotation (Line(
            points={{-12,-74},{2,-74},{2,7},{0,7}}, color={191,0,0}));
      connect(prescribedHeatFlowRad.port, OfficeCell.gainRad) annotation (Line(
            points={{-12,-92},{4,-92},{4,4},{0,4}}, color={191,0,0}));
      connect(boundary.ports[1], OfficeCell.port_b) annotation (Line(points={{
              58,-54},{24,-54},{24,20},{-12,20}}, color={0,127,255}));
      connect(ventIPd.y, boundary.m_flow_in) annotation (Line(points={{103.5,-49},{92.5,
              -49},{92.5,-46},{80,-46}}, color={0,0,127}));
      connect(boundaryWall1.T, OfficeCell.TSensor) annotation (Line(points={{-81,
              30},{-40,30},{-40,12},{1,12}}, color={0,0,127}));
      connect(boundaryWall2.T, OfficeCell.TSensor) annotation (Line(points={{-79,
              0},{-38,0},{-38,12},{1,12}}, color={0,0,127}));
      connect(boundaryWall.T, OfficeCell.TSensor) annotation (Line(points={{-8,
              69},{-4,69},{-4,12},{1,12}}, color={0,0,127}));
      connect(ventIPd.y, ventQ.ventCtrl) annotation (Line(points={{103.5,-49},{89.75,
              -49},{89.75,-61.76},{90,-61.76}}, color={0,0,127}));
      connect(floor.propsBus_b, plenum.propsBus[1]) annotation (Line(
          points={{-112,15},{-124,15},{-124,7.66667},{-152,7.66667}},
          color={255,204,51},
          thickness=0.5));
      connect(outerWall1.propsBus_a, plenum.propsBus[2]) annotation (Line(
          points={{-150,-15},{-150,-2.5},{-152,-2.5},{-152,7}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall3.propsBus_a, plenum.propsBus[3]) annotation (Line(
          points={{-169,-6},{-160,-6},{-160,6.33333},{-152,6.33333}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall3.T, plenum.TSensor) annotation (Line(points={{-185,-6},
              {-156,-6},{-156,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall4.T, plenum.TSensor) annotation (Line(points={{-185,
              24},{-154,24},{-154,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall4.propsBus_a, plenum.propsBus[4]) annotation (Line(
          points={{-169,24},{-169,25},{-152,25},{-152,5.66667}},
          color={255,204,51},
          thickness=0.5));
      connect(boundaryWall5.T, plenum.TSensor) annotation (Line(points={{-140,
              47},{-132,47},{-132,4},{-131,4}}, color={0,0,127}));
      connect(boundaryWall5.propsBus_a, plenum.propsBus[5]) annotation (Line(
          points={{-140,31},{-144,31},{-144,5},{-152,5}},
          color={255,204,51},
          thickness=0.5));
      connect(floor1.propsBus_a, OfficeCell.propsBus[7]) annotation (Line(
          points={{-108,-15},{-108,-18},{-90,-18},{-90,10},{-56,10},{-56,
              12.2857},{-20,12.2857}},
          color={255,204,51},
          thickness=0.5));
      connect(floor1.propsBus_b, plenum.propsBus[6]) annotation (Line(
          points={{-108,-5},{-108,-1.5},{-152,-1.5},{-152,4.33333}},
          color={255,204,51},
          thickness=0.5));
      connect(OfficeCell.propsBus[5], window.propsBus_a)
        annotation (Line(
          points={{-20,13.4286},{-34,13.4286},{-34,-25}},
          color={255,204,51},
          thickness=0.5));
      connect(window.Ctrl, screenControl1_1.y) annotation (Line(points={{-46,-34},{-59.36,
              -34}},                       color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,100}})),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{200,
                100}})),
        experiment(
          StopTime=31536000,
          Interval=600,
          Tolerance=1e-06,
          __Dymola_Algorithm="Lsodar"),
        __Dymola_Commands(file=
              "Resources/Scripts/Dymola/Examples/Tutorial/Example1.mos"
            "Simulate and plot"),
        Documentation(info="<html>
<p>
This first example file instantiates a simple building model.
</p>
</html>",     revisions="<html>
<ul>
<li>
September 18, 2019 by Filip Jorissen:<br/>
First implementation for the IDEAS crash course.
</li>
</ul>
</html>"),
        __Dymola_experimentSetupOutput(events=false));
    end OffZave1;

    package IrrModels

      model MultiFacW

        extends Modelica.Blocks.Icons.Block;

        parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
        parameter Modelica.SIunits.Length winW = 2.7 "Width of subsurface";
        parameter Modelica.SIunits.Length winH = 1.4 "Height of subsurface";
        parameter Modelica.SIunits.Length winl = 5+39 "Horizontal distance of subsurface edge from facade edge";
        parameter Modelica.SIunits.Length winz = 1.5+27 "Vertical distance of subsurface edge from ground";
        parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
        /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
        parameter Real res(min=0)=10 "Resolution of descritization; =1 for 1cm";
        constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
        parameter Integer nS(min=1)=23 "No. of shading objects";
        final parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane" annotation(HideResult=true);
        final parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis" annotation(HideResult=true);

        Real facdif;
        Real facdir;

      protected
        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-138,78},{-118,98}})));
        Real FinalSVF1[n*k];
        Real FinalSVF3[n*k];
        Real FinalSVF[n*k];
        Real FinalI[n*k];
        IRR.Irr.Components_Irr.ShObjW shObjW1(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=23+0,
          Shade_Face_azi=1.9896753472735,
          Shade_Face_D=143+0+0,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-88,110},{-68,130}})));
        IRR.Irr.Components_Irr.ShObjW shObjW2(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=23+0,
          Shade_Face_azi=0.41887902047864,
          Shade_Face_D=143+0+0,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-82,106},{-62,126}})));
        IRR.Irr.Components_Irr.ShObjW shObjW3(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=23+0,
          Shade_Face_azi=-0.5235987755983,
          Shade_Face_D=158+0+0,
          Shade_Face_theta=2.2165681500328)
          annotation (Placement(transformation(extent={{-94,98},{-74,118}})));
        IRR.Irr.Components_Irr.ShObjW shObjW4(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=27+0,
          Shade_Face_azi=1.6929693744345,
          Shade_Face_D=110+0+0,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-64,78},{-44,98}})));
        IRR.Irr.Components_Irr.ShObjW shObjW5(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=27+0,
          Shade_Face_azi=0.1221730476396,
          Shade_Face_D=110+0+0,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-72,72},{-52,92}})));
        IRR.Irr.Components_Irr.ShObjW shObjW6(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=27+0,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=126+0+0,
          Shade_Face_theta=1.9547687622336)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW7(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=31+0,
          Shade_Face_azi=1.3788101090755,
          Shade_Face_D=77+0+0,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-110,58},{-90,78}})));
        IRR.Irr.Components_Irr.ShObjW shObjW8(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=31+0,
          Shade_Face_azi=-0.19198621771938,
          Shade_Face_D=77+0+0,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-104,52},{-84,72}})));
        IRR.Irr.Components_Irr.ShObjW shObjW9(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=31+0,
          Shade_Face_azi=-1.1170107212764,
          Shade_Face_D=93+0+0,
          Shade_Face_theta=1.6057029118348)
          annotation (Placement(transformation(extent={{-114,46},{-94,66}})));
        IRR.Irr.Components_Irr.ShObjW shObjW10(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=34+0,
          Shade_Face_azi=1.1519173063163,
          Shade_Face_D=53+0+0,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-78,30},{-58,50}})));
        IRR.Irr.Components_Irr.ShObjW shObjW11(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=34+0,
          Shade_Face_azi=-0.43633231299858,
          Shade_Face_D=53+0+0,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-66,22},{-46,42}})));
        IRR.Irr.Components_Irr.ShObjW shObjW12(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=34+0,
          Shade_Face_azi=-1.3788101090755,
          Shade_Face_D=69+0+0,
          Shade_Face_theta=1.0821041362365)
          annotation (Placement(transformation(extent={{-74,14},{-54,34}})));
        IRR.Irr.Components_Irr.ShObjW shObjW13(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=25,
          Shade_Face_H=34+0,
          Shade_Face_azi=0.17453292519943,
          Shade_Face_D=95+0+0,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-92,6},{-72,26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW14(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=24,
          Shade_Face_H=34+0,
          Shade_Face_azi=3.0019663134302,
          Shade_Face_D=95+0+0,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-82,-2},{-62,18}})));
        IRR.Irr.Components_Irr.ShObjW shObjW15(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=14,
          Shade_Face_H=34+0,
          Shade_Face_azi=-0.41887902047864,
          Shade_Face_D=53+0+0,
          Shade_Face_theta=1.2915436464758)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        IRR.Irr.Components_Irr.ShObjW shObjW16(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=38+0,
          Shade_Face_azi=-2.2340214425527,
          Shade_Face_D=44+0+0,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-100,-46},{-80,-26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW17(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=38+0,
          Shade_Face_azi=-0.034906585039887,
          Shade_Face_D=81+0+0,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-88,-52},{-68,-32}})));
        IRR.Irr.Components_Irr.ShObjW shObjW18(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+0,
          Shade_Face_azi=2.7925268031909,
          Shade_Face_D=81+0+0,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
        IRR.Irr.Components_Irr.ShObjW shObjW19(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+0,
          Shade_Face_azi=-0.62831853071796,
          Shade_Face_D=44+0+0,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
        IRR.Irr.Components_Irr.ShObjW shObjW20(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+0,
          Shade_Face_azi=-2.3736477827123,
          Shade_Face_D=67+0+0,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-72,-96},{-52,-76}})));
        IRR.Irr.Components_Irr.ShObjW shObjW21(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=28,
          Shade_Face_H=42+0,
          Shade_Face_azi=-0.20943951023932,
          Shade_Face_D=91+0+0,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-60,-102},{-40,-82}})));
        IRR.Irr.Components_Irr.ShObjW shObjW22(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+0,
          Shade_Face_azi=2.5481807079117,
          Shade_Face_D=91+0+0,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-84,-110},{-64,-90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW23(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=42+0,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=67+0+0,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-68,-116},{-48,-96}})));

      equation

        for i in 1:n*k loop
          /*Add shading object "DifShObj1,DifShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalSVF1[i] =1 - nS + (shObjW1.SVF1[i] +
            shObjW2.SVF1[i] + shObjW3.SVF1[i] + shObjW4.SVF1[i] + shObjW5.SVF1[i] + shObjW6.SVF1[i] +
            shObjW7.SVF1[i] + shObjW8.SVF1[i] + shObjW9.SVF1[i] + shObjW10.SVF1[i] + shObjW11.SVF1[i]
             + shObjW12.SVF1[i] + shObjW13.SVF1[i] + shObjW14.SVF1[i] + shObjW15.SVF1[i] +
            shObjW16.SVF1[i] + shObjW17.SVF1[i] + shObjW18.SVF1[i] + shObjW19.SVF1[i] + shObjW20.SVF1[
            i] + shObjW21.SVF1[i] + shObjW22.SVF1[i] + shObjW23.SVF1[i]);

          /*Add shading object "ShObj1,ShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalI[i] =IRR.Irr.Components_Irr.Functions_Irr.FinalI((shObjW1.I[i] +
            shObjW2.I[i] + shObjW3.I[i] + shObjW4.I[i] + shObjW5.I[i] + shObjW6.I[i] +
            shObjW7.I[i] + shObjW8.I[i] + shObjW9.I[i] + shObjW10.I[i] + shObjW11.I[i]
             + shObjW12.I[i] + shObjW13.I[i] + shObjW14.I[i] + shObjW15.I[i] +
            shObjW16.I[i] + shObjW17.I[i] + shObjW18.I[i] + shObjW19.I[i] + shObjW20.I[
            i] + shObjW21.I[i] + shObjW22.I[i] + shObjW23.I[i])/nS);

          FinalSVF3[i] =max(0,1 - nS + (shObjW1.SVF3[i] +
            shObjW2.SVF3[i] + shObjW3.SVF3[i] + shObjW4.SVF3[i] + shObjW5.SVF3[i] + shObjW6.SVF3[i] +
            shObjW7.SVF3[i] + shObjW8.SVF3[i] + shObjW9.SVF3[i] + shObjW10.SVF3[i] + shObjW11.SVF3[i]
             + shObjW12.SVF3[i] + shObjW13.SVF3[i] + shObjW14.SVF3[i] + shObjW15.SVF3[i] +
            shObjW16.SVF3[i] + shObjW17.SVF3[i] + shObjW18.SVF3[i] + shObjW19.SVF3[i] + shObjW20.SVF3[
            i] + shObjW21.SVF3[i] + shObjW22.SVF3[i] + shObjW23.SVF3[i]));

          FinalSVF[i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalI[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);

        end for;

        facdif = sum(FinalSVF)/(n*k);
        facdir = sum(FinalI)/(n*k);

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                  -100},{100,100}})),                                  Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-140,-100},{100,100}}), graphics={
              Line(
                points={{0,0},{0,100}},
                color={238,46,47},
                thickness=0.5),
              Text(
                extent={{-140,6},{-116,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="WEST
[+90 deg]"),  Text(
                extent={{-18,112},{28,82}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="NORTH [+/-180 deg]"),
              Text(
                extent={{78,6},{100,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="EAST
[-90 deg]"),  Text(
                extent={{-14,-86},{22,-108}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="SOUTH [0 deg]")}),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600,
            __Dymola_Algorithm="Euler"));
      end MultiFacW;

      model GloMultiFacW

        extends Modelica.Blocks.Icons.Block;

        parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
        parameter Modelica.SIunits.Length winW = 3 "Width of subsurface";
        parameter Modelica.SIunits.Length winH = 3 "Height of subsurface";
        parameter Modelica.SIunits.Length winl = 44 "Horizontal distance of subsurface edge from facade edge";
        parameter Modelica.SIunits.Length winz = 3 "Vertical distance of subsurface edge from ground";
        parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
        /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
        parameter Real res(min=0)=100 "Resolution of descritization; =1 for 1cm";
        constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
        parameter Integer nS(min=1)=23 "No. of shading objects";
        final parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane" annotation(HideResult=true);
        final parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis" annotation(HideResult=true);

        //Real SkyDifSH[n*k](min=0,unit="W/m2");
        //Real DirSH[n*k](min=0,unit="W/m2");
        //Real GloSH[n*k](min=0,unit="W/m2");



        Modelica.Blocks.Continuous.Integrator Dir(k=1/(60*60*1000), u=AvgDirSH)
          annotation (Placement(transformation(extent={{74,52},{94,72}})));
        Modelica.Blocks.Continuous.Integrator SDif(k=1/(60*60*1000), u=AvgSkyDifSH)
          annotation (Placement(transformation(extent={{108,20},{128,40}})));
        Modelica.Blocks.Continuous.Integrator Glo(k=1/(60*60*1000), u=AvgGloSH)
          annotation (Placement(transformation(extent={{110,74},{130,94}})));

        Real SkyDifIrr(min=0,unit="W/m2");
        Real GroDifIrr(min=0,unit="W/m2");
        Real DirIrr(min=0,unit="W/m2");
        Real GloIrr(min=0,unit="W/m2");
        Real AvgDirSH(min=0,unit="W/m2");
        Real AvgSkyDifSH(min=0,unit="W/m2");
        Real AvgGloSH(min=0,unit="W/m2");

        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-138,80},{-118,100}})));
        Real FinalSVF1[n*k];
        Real FinalSVF3[n*k];
        Real FinalSVF[n*k];
        Real facskydif;
        Real FinalI[n*k];
        Real facdir;
      protected
        IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
          til=TgtFace_tilt,
          lat=lat,
          azi=TgtFace_azi)
          annotation (Placement(transformation(extent={{-144,34},{-164,54}})));

        IRR.Irr.Components_Irr.DiffusePerez HDifTil(
          til=TgtFace_tilt,
          lat=lat,
          azi=TgtFace_azi,
          outSkyCon=true,
          outGroCon=true)                           annotation (Placement(
              transformation(
              extent={{-6,-6},{6,6}},
              rotation=180,
              origin={-152,74})));


        IRR.Irr.Components_Irr.ShObjW shObjW1(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=23+00,
          Shade_Face_azi=1.9896753472735,
          Shade_Face_D=143+0-40,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-88,110},{-68,130}})));
        IRR.Irr.Components_Irr.ShObjW shObjW2(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=23+00,
          Shade_Face_azi=0.41887902047864,
          Shade_Face_D=143+0-80,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-82,106},{-62,126}})));
        IRR.Irr.Components_Irr.ShObjW shObjW3(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=23+00,
          Shade_Face_azi=-0.5235987755983,
          Shade_Face_D=158+0-80,
          Shade_Face_theta=2.2165681500328)
          annotation (Placement(transformation(extent={{-94,98},{-74,118}})));
        IRR.Irr.Components_Irr.ShObjW shObjW4(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=27+00,
          Shade_Face_azi=1.6929693744345,
          Shade_Face_D=110+0-80,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-64,78},{-44,98}})));
        IRR.Irr.Components_Irr.ShObjW shObjW5(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=27+00,
          Shade_Face_azi=0.1221730476396,
          Shade_Face_D=110+0-80,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-72,72},{-52,92}})));
        IRR.Irr.Components_Irr.ShObjW shObjW6(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=27+00,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=126+0-80,
          Shade_Face_theta=1.9547687622336)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW7(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=31+00,
          Shade_Face_azi=1.3788101090755,
          Shade_Face_D=77+0-50,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-110,58},{-90,78}})));
        IRR.Irr.Components_Irr.ShObjW shObjW8(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=31+00,
          Shade_Face_azi=-0.19198621771938,
          Shade_Face_D=77+0-60,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-104,52},{-84,72}})));
        IRR.Irr.Components_Irr.ShObjW shObjW9(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=31+00,
          Shade_Face_azi=-1.1170107212764,
          Shade_Face_D=93+0-80,
          Shade_Face_theta=1.6057029118348)
          annotation (Placement(transformation(extent={{-114,46},{-94,66}})));
        IRR.Irr.Components_Irr.ShObjW shObjW10(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=34+00,
          Shade_Face_azi=1.1519173063163,
          Shade_Face_D=53+0-50,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-78,30},{-58,50}})));
        IRR.Irr.Components_Irr.ShObjW shObjW11(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=34+00,
          Shade_Face_azi=-0.43633231299858,
          Shade_Face_D=53+0-50,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-66,22},{-46,42}})));
        IRR.Irr.Components_Irr.ShObjW shObjW12(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=34+00,
          Shade_Face_azi=-1.3788101090755,
          Shade_Face_D=69+0-60,
          Shade_Face_theta=1.0821041362365)
          annotation (Placement(transformation(extent={{-74,14},{-54,34}})));
        IRR.Irr.Components_Irr.ShObjW shObjW13(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=25,
          Shade_Face_H=34+00,
          Shade_Face_azi=0.17453292519943,
          Shade_Face_D=95+0-80,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-92,6},{-72,26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW14(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=24,
          Shade_Face_H=34+00,
          Shade_Face_azi=3.0019663134302,
          Shade_Face_D=95+0-80,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-82,-2},{-62,18}})));
        IRR.Irr.Components_Irr.ShObjW shObjW15(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=14,
          Shade_Face_H=34+00,
          Shade_Face_azi=-0.41887902047864,
          Shade_Face_D=53+0-50,
          Shade_Face_theta=1.2915436464758)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        IRR.Irr.Components_Irr.ShObjW shObjW16(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=38+00,
          Shade_Face_azi=-2.2340214425527,
          Shade_Face_D=44+0-40,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-100,-46},{-80,-26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW17(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=38+00,
          Shade_Face_azi=-0.034906585039887,
          Shade_Face_D=81+0-60,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-88,-52},{-68,-32}})));
        IRR.Irr.Components_Irr.ShObjW shObjW18(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+00,
          Shade_Face_azi=2.7925268031909,
          Shade_Face_D=81+0-80,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
        IRR.Irr.Components_Irr.ShObjW shObjW19(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+00,
          Shade_Face_azi=-0.62831853071796,
          Shade_Face_D=44+0-40,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
        IRR.Irr.Components_Irr.ShObjW shObjW20(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+00,
          Shade_Face_azi=-2.3736477827123,
          Shade_Face_D=67+0-60,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-72,-96},{-52,-76}})));
        IRR.Irr.Components_Irr.ShObjW shObjW21(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=28,
          Shade_Face_H=42+00,
          Shade_Face_azi=-0.20943951023932,
          Shade_Face_D=91+0-80,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-60,-102},{-40,-82}})));
        IRR.Irr.Components_Irr.ShObjW shObjW22(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+00,
          Shade_Face_azi=2.5481807079117,
          Shade_Face_D=91+0-80,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-84,-110},{-64,-90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW23(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=42+00,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=67+0-60,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-68,-116},{-48,-96}})));

      equation

        SkyDifIrr = HDifTil.HSkyDifTil;
        GroDifIrr = HDifTil.HGroDifTil;
        DirIrr = HDirTil.H;
        GloIrr = DirIrr+SkyDifIrr+GroDifIrr;

        for i in 1:n*k loop
          /*Add shading object "DifShObj1,DifShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalSVF1[i] =max(0,1 - nS + (shObjW1.SVF1[i] +
            shObjW2.SVF1[i] + shObjW3.SVF1[i] + shObjW4.SVF1[i] + shObjW5.SVF1[i] + shObjW6.SVF1[i] +
            shObjW7.SVF1[i] + shObjW8.SVF1[i] + shObjW9.SVF1[i] + shObjW10.SVF1[i] + shObjW11.SVF1[i]
             + shObjW12.SVF1[i] + shObjW13.SVF1[i] + shObjW14.SVF1[i] + shObjW15.SVF1[i] +
            shObjW16.SVF1[i] + shObjW17.SVF1[i] + shObjW18.SVF1[i] + shObjW19.SVF1[i] + shObjW20.SVF1[
            i] + shObjW21.SVF1[i] + shObjW22.SVF1[i] + shObjW23.SVF1[i]));

          /*Add shading object "ShObj1,ShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalI[i] =IRR.Irr.Components_Irr.Functions_Irr.FinalI((shObjW1.I[i] +
            shObjW2.I[i] + shObjW3.I[i] + shObjW4.I[i] + shObjW5.I[i] + shObjW6.I[i] +
            shObjW7.I[i] + shObjW8.I[i] + shObjW9.I[i] + shObjW10.I[i] + shObjW11.I[i]
             + shObjW12.I[i] + shObjW13.I[i] + shObjW14.I[i] + shObjW15.I[i] +
            shObjW16.I[i] + shObjW17.I[i] + shObjW18.I[i] + shObjW19.I[i] + shObjW20.I[
            i] + shObjW21.I[i] + shObjW22.I[i] + shObjW23.I[i])/nS);

          FinalSVF3[i] =max(0,1 - nS + (shObjW1.SVF3[i] +
            shObjW2.SVF3[i] + shObjW3.SVF3[i] + shObjW4.SVF3[i] + shObjW5.SVF3[i] + shObjW6.SVF3[i] +
            shObjW7.SVF3[i] + shObjW8.SVF3[i] + shObjW9.SVF3[i] + shObjW10.SVF3[i] + shObjW11.SVF3[i]
             + shObjW12.SVF3[i] + shObjW13.SVF3[i] + shObjW14.SVF3[i] + shObjW15.SVF3[i] +
            shObjW16.SVF3[i] + shObjW17.SVF3[i] + shObjW18.SVF3[i] + shObjW19.SVF3[i] + shObjW20.SVF3[
            i] + shObjW21.SVF3[i] + shObjW22.SVF3[i] + shObjW23.SVF3[i]));

          FinalSVF[i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalI[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);

          //DirSH[i] = FinalI[i]*DirIrr;
          //SkyDifSH[i] = FinalSVF[i]*SkyDifIrr;
          //GloSH[i] = DirSH[i]+SkyDifSH[i]+GroDifIrr;

        end for;

        facskydif = sum(FinalSVF)/(n*k);
        AvgSkyDifSH = facskydif*SkyDifIrr;
        facdir = sum(FinalI)/(n*k);
        AvgDirSH = facdir*DirIrr;
        AvgGloSH = AvgDirSH+AvgSkyDifSH+GroDifIrr;

        connect(sim.weaDatBus, HDifTil.weaBus) annotation (Line(
            points={{-118.1,90},{-130,90},{-130,74},{-146,74}},
            color={255,204,51},
            thickness=0.5));
        connect(sim.weaDatBus, HDirTil.weaBus) annotation (Line(
            points={{-118.1,90},{-130,90},{-130,44},{-144,44}},
            color={255,204,51},
            thickness=0.5));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                  -100},{100,100}})),                                  Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-140,-100},{100,100}}), graphics={
              Line(
                points={{0,0},{0,100}},
                color={238,46,47},
                thickness=0.5),
              Text(
                extent={{-140,6},{-116,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="WEST
[+90 deg]"),  Text(
                extent={{-18,112},{28,82}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="NORTH [+/-180 deg]"),
              Text(
                extent={{78,6},{100,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="EAST
[-90 deg]"),  Text(
                extent={{-14,-86},{22,-108}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="SOUTH [0 deg]")}),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600,
            __Dymola_Algorithm="Euler"),
          __Dymola_experimentSetupOutput(events=false));
      end GloMultiFacW;

      model HorIrr

        Real SkyDifIrr(min=0,unit="W/m2");
        Real GroDifIrr(min=0,unit="W/m2");
        Real DirIrr(min=0,unit="W/m2");
        Real GloIrr(min=0,unit="W/m2");

        Modelica.Blocks.Continuous.Integrator Dir(k=1/(60*60*1000), u=DirIrr)
          annotation (Placement(transformation(extent={{74,52},{94,72}})));
        Modelica.Blocks.Continuous.Integrator SDif(k=1/(60*60*1000), u=
              SkyDifIrr)
          annotation (Placement(transformation(extent={{108,20},{128,40}})));
        Modelica.Blocks.Continuous.Integrator Glo(k=1/(60*60*1000), u=GloIrr)
          annotation (Placement(transformation(extent={{110,74},{130,94}})));
      protected
        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-70,80},{-50,100}})));
        IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
          til=0,
          lat=0.89011791851711,
          azi=0.75049157835756)
          annotation (Placement(transformation(extent={{-76,34},{-96,54}})));
        IRR.Irr.Components_Irr.DiffusePerez HDifTil(
          til=0,
          lat=0.89011791851711,
          azi=0.75049157835756,
          outSkyCon=true,
          outGroCon=true)                           annotation (Placement(
              transformation(
              extent={{-6,-6},{6,6}},
              rotation=180,
              origin={-84,74})));
      equation

        SkyDifIrr = HDifTil.HSkyDifTil;
        GroDifIrr = HDifTil.HGroDifTil;
        DirIrr = HDirTil.H;
        GloIrr = DirIrr+SkyDifIrr+GroDifIrr;

        connect(sim.weaDatBus,HDifTil. weaBus) annotation (Line(
            points={{-50.1,90},{-62,90},{-62,74},{-78,74}},
            color={255,204,51},
            thickness=0.5));
        connect(sim.weaDatBus,HDirTil. weaBus) annotation (Line(
            points={{-50.1,90},{-62,90},{-62,44},{-76,44}},
            color={255,204,51},
            thickness=0.5));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false)),
          Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600,
            __Dymola_Algorithm="Euler"));
      end HorIrr;
    end IrrModels;

    package IrrModel60

      package PartialBIPVdemo
        model ShObjW60 "Model for sub-surface; single shading facade"

          extends Modelica.Blocks.Icons.Block;

          parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude" annotation(HideResult=true);

          parameter Modelica.SIunits.Length winW = 0.9 "Width of subsurface";
          parameter Modelica.SIunits.Length winH = 1.5 "Height of subsurface";
          parameter Modelica.SIunits.Length winl = 10 "Horizontal distance of subsurface edge from facade edge";
          parameter Modelica.SIunits.Length winz = 0 "Vertical distance of subsurface edge from ground";
          parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle" annotation(HideResult=true);
          /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
          parameter Real res(min=0)=10 "Resolution of descritization; =1 for 1cm" annotation(HideResult=true);
          parameter Modelica.SIunits.Length Shade_Face_len(min=0)=30 "Length of facade";
          parameter Modelica.SIunits.Length Shade_Face_H(min=0)=30 "Height of facade";
          parameter Modelica.SIunits.Angle Shade_Face_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
          /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
          parameter Modelica.SIunits.Length Shade_Face_D(min=0)=15 "Distance between target and shadow facade origins [See Figure] ";
          /* Ensure shading facade does not cross North Axis */
          parameter Modelica.SIunits.Angle Shade_Face_theta(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(30)
          "Angular orientation of line connecting target and shadow facade start-points [See Figure]";
          //Real I[n*k];
          Modelica.SIunits.Length H_sh[n];

        //protected
          parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane";
          parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis";

          Modelica.SIunits.Length x0y0[n,2];
          //Modelica.SIunits.Angle az1[n];
          Modelica.SIunits.Angle Sun_Az1[6] = sim.radSol.solAzi.solAzi, Sun_Zen1[6] = sim.radSol.angZen;

          inner IRR.Irr.Components_Irr.SimInfoManager sim
            annotation (Placement(transformation(extent={{-98,78},{-78,98}})));
        equation

          for i in 1:n loop

            x0y0[i,1] = Shade_Face_D*Modelica.Math.cos(-Modelica.Constants.pi/2+Shade_Face_theta)
            - (winl + ((i-1)*res/100))*Modelica.Math.cos(TgtFace_azi);
            x0y0[i,2] = Shade_Face_D*Modelica.Math.sin(-Modelica.Constants.pi/2+Shade_Face_theta)
            - (winl + ((i-1)*res/100))*Modelica.Math.sin(TgtFace_azi);
            //az0[i] =Functions_Dir.tanInv(x0y0[i,1], x0y0[i,2]);
            //az1[i] =Functions_Dir.tanInv(x0y0[i,1] + Shade_Face_len*Modelica.Math.cos(Shade_Face_azi),
            //x0y0[i,2] + Shade_Face_len*Modelica.Math.sin(Shade_Face_azi));

            H_sh[i] =IRR.Irr.Components_Irr.Functions_Irr.H_SH(
                    IRR.Irr.Components_Irr.Functions_Irr.distDir(
                      Sun_Az1[1],
                      Shade_Face_azi,
                      IRR.Irr_Dir.Components_Dir.Functions_Dir.tanInv(x0y0[i, 1],
                  x0y0[i, 2]),
                      IRR.Irr_Dir.Components_Dir.Functions_Dir.tanInv(x0y0[i, 1]
                   + Shade_Face_len*Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2]
                   + Shade_Face_len*Modelica.Math.sin(Shade_Face_azi)),
                      TgtFace_azi,
                      IRR.Irr_Dir.Components_Dir.Functions_Dir.Di(
                        IRR.Irr_Dir.Components_Dir.Functions_Dir.tanInv(x0y0[i, 1],
                    x0y0[i, 2]),
                        Shade_Face_D,
                        Shade_Face_theta,
                        i,
                        res,
                        TgtFace_azi)),
                    Shade_Face_H,
                    Sun_Zen1[1]);

        /*  for j in 1:k loop

      I[(i - 1)*k + j] = IRR.Irr_Dir.Components_Dir.Functions_Dir.Frac(H_sh[i],
        winz + ((j - 1)*res/100));

    end for; */

          end for;

           annotation (experiment(
              StartTime=8640000,
              StopTime=9072000,
              Interval=599.999616,
              __Dymola_fixedstepsize=600,
              __Dymola_Algorithm="Euler"));
        end ShObjW60;

      end PartialBIPVdemo;

      model MultiFacW60

        extends Modelica.Blocks.Icons.Block;

        parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
        parameter Modelica.SIunits.Length winW = 0.9-0.15 "Width of subsurface";
        parameter Modelica.SIunits.Length winH = 1.5-0.15 "Height of subsurface";
        parameter Modelica.SIunits.Length winl = 44+((2*0.9)+(0.15/2)) "Horizontal distance of subsurface edge from facade edge";
        parameter Modelica.SIunits.Length winz = 3+(0.15/2) "Vertical distance of subsurface edge from ground";
        parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
        /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
        parameter Real res(min=0)=15 "Resolution of descritization; =1 for 1cm";
        constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
        parameter Integer nS(min=1)=23 "No. of shading objects";
        final parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane" annotation(HideResult=true);
        final parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis" annotation(HideResult=true);

        Real facdif;
        Real facdir;
        Real FinalSVF[n*k];
        Real FinalI[n*k];

        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-138,78},{-118,98}})));
      protected
        Real FinalSVF1[n*k];
        Real FinalSVF2[n*k];
        Real FinalSVF3[n*k];
        IRR.Irr.Components_Irr.ShObjW shObjW1(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=23+0,
          Shade_Face_azi=1.9896753472735,
          Shade_Face_D=143+0-0,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-88,110},{-68,130}})));
        IRR.Irr.Components_Irr.ShObjW shObjW2(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=23+0,
          Shade_Face_azi=0.41887902047864,
          Shade_Face_D=143+0-0,
          Shade_Face_theta=2.2514747350727)
          annotation (Placement(transformation(extent={{-82,106},{-62,126}})));
        IRR.Irr.Components_Irr.ShObjW shObjW3(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=23+0,
          Shade_Face_azi=-0.5235987755983,
          Shade_Face_D=158+0-0,
          Shade_Face_theta=2.2165681500328)
          annotation (Placement(transformation(extent={{-94,98},{-74,118}})));
        IRR.Irr.Components_Irr.ShObjW shObjW4(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=27+0,
          Shade_Face_azi=1.6929693744345,
          Shade_Face_D=110+0-0,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-64,78},{-44,98}})));
        IRR.Irr.Components_Irr.ShObjW shObjW5(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=27+0,
          Shade_Face_azi=0.1221730476396,
          Shade_Face_D=110+0-0,
          Shade_Face_theta=1.9896753472735)
          annotation (Placement(transformation(extent={{-72,72},{-52,92}})));
        IRR.Irr.Components_Irr.ShObjW shObjW6(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=27+0,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=126+0-0,
          Shade_Face_theta=1.9547687622336)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW7(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=31+0,
          Shade_Face_azi=1.3788101090755,
          Shade_Face_D=77+0-0,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-110,58},{-90,78}})));
        IRR.Irr.Components_Irr.ShObjW shObjW8(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=31+0,
          Shade_Face_azi=-0.19198621771938,
          Shade_Face_D=77+0-0,
          Shade_Face_theta=1.6580627893946)
          annotation (Placement(transformation(extent={{-104,52},{-84,72}})));
        IRR.Irr.Components_Irr.ShObjW shObjW9(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=31+0,
          Shade_Face_azi=-1.1170107212764,
          Shade_Face_D=93+0-0,
          Shade_Face_theta=1.6057029118348)
          annotation (Placement(transformation(extent={{-114,46},{-94,66}})));
        IRR.Irr.Components_Irr.ShObjW shObjW10(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=34+0,
          Shade_Face_azi=1.1519173063163,
          Shade_Face_D=53+0-0,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-78,30},{-58,50}})));
        IRR.Irr.Components_Irr.ShObjW shObjW11(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=16,
          Shade_Face_H=34+0,
          Shade_Face_azi=-0.43633231299858,
          Shade_Face_D=53+0-0,
          Shade_Face_theta=1.0646508437165)
          annotation (Placement(transformation(extent={{-66,22},{-46,42}})));
        IRR.Irr.Components_Irr.ShObjW shObjW12(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=18,
          Shade_Face_H=34+0,
          Shade_Face_azi=-1.3788101090755,
          Shade_Face_D=69+0-0,
          Shade_Face_theta=1.0821041362365)
          annotation (Placement(transformation(extent={{-74,14},{-54,34}})));
        IRR.Irr.Components_Irr.ShObjW shObjW13(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=25,
          Shade_Face_H=34+0,
          Shade_Face_azi=0.17453292519943,
          Shade_Face_D=95+0-0,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-92,6},{-72,26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW14(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=24,
          Shade_Face_H=34+0,
          Shade_Face_azi=3.0019663134302,
          Shade_Face_D=95+0-0,
          Shade_Face_theta=1.3089969389957)
          annotation (Placement(transformation(extent={{-82,-2},{-62,18}})));
        IRR.Irr.Components_Irr.ShObjW shObjW15(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=14,
          Shade_Face_H=34+0,
          Shade_Face_azi=-0.41887902047864,
          Shade_Face_D=53+0-0,
          Shade_Face_theta=1.2915436464758)
          annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
        IRR.Irr.Components_Irr.ShObjW shObjW16(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=12,
          Shade_Face_H=38+0,
          Shade_Face_azi=-2.2340214425527,
          Shade_Face_D=44+0-0,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-100,-46},{-80,-26}})));
        IRR.Irr.Components_Irr.ShObjW shObjW17(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=38+0,
          Shade_Face_azi=-0.034906585039887,
          Shade_Face_D=81+0-0,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-88,-52},{-68,-32}})));
        IRR.Irr.Components_Irr.ShObjW shObjW18(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+0,
          Shade_Face_azi=2.7925268031909,
          Shade_Face_D=81+0-0,
          Shade_Face_theta=0.69813170079773)
          annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
        IRR.Irr.Components_Irr.ShObjW shObjW19(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=21,
          Shade_Face_H=38+0,
          Shade_Face_azi=-0.62831853071796,
          Shade_Face_D=44+0-0,
          Shade_Face_theta=0.34906585039887)
          annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
        IRR.Irr.Components_Irr.ShObjW shObjW20(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+0,
          Shade_Face_azi=-2.3736477827123,
          Shade_Face_D=67+0-0,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-72,-96},{-52,-76}})));
        IRR.Irr.Components_Irr.ShObjW shObjW21(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=28,
          Shade_Face_H=42+0,
          Shade_Face_azi=-0.20943951023932,
          Shade_Face_D=91+0-0,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-60,-102},{-40,-82}})));
        IRR.Irr.Components_Irr.ShObjW shObjW22(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=19,
          Shade_Face_H=42+0,
          Shade_Face_azi=2.5481807079117,
          Shade_Face_D=91+0-0,
          Shade_Face_theta=0.10471975511966)
          annotation (Placement(transformation(extent={{-84,-110},{-64,-90}})));
        IRR.Irr.Components_Irr.ShObjW shObjW23(
          lat=lat,
          winW=winW,
          winH=winH,
          winl=winl,
          winz=winz,
          TgtFace_azi=TgtFace_azi,
          res=res,
          Shade_Face_len=26,
          Shade_Face_H=42+0,
          Shade_Face_azi=-0.80285145591739,
          Shade_Face_D=67+0-0,
          Shade_Face_theta=2.757620218151)
          annotation (Placement(transformation(extent={{-68,-116},{-48,-96}})));

      equation

        for i in 1:60 loop
          /*Add shading object "DifShObj1,DifShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalSVF1[i] =1 - nS + (shObjW1.SVF1[i] +
            shObjW2.SVF1[i] + shObjW3.SVF1[i] + shObjW4.SVF1[i] + shObjW5.SVF1[i] + shObjW6.SVF1[i] +
            shObjW7.SVF1[i] + shObjW8.SVF1[i] + shObjW9.SVF1[i] + shObjW10.SVF1[i] + shObjW11.SVF1[i]
             + shObjW12.SVF1[i] + shObjW13.SVF1[i] + shObjW14.SVF1[i] + shObjW15.SVF1[i] +
            shObjW16.SVF1[i] + shObjW17.SVF1[i] + shObjW18.SVF1[i] + shObjW19.SVF1[i] + shObjW20.SVF1[
            i] + shObjW21.SVF1[i] + shObjW22.SVF1[i] + shObjW23.SVF1[i]);

          /*Add shading object "ShObj1,ShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
          FinalSVF2[i] =IRR.Irr.Components_Irr.Functions_Irr.FinalI((shObjW1.I[i] +
            shObjW2.I[i] + shObjW3.I[i] + shObjW4.I[i] + shObjW5.I[i] + shObjW6.I[i] +
            shObjW7.I[i] + shObjW8.I[i] + shObjW9.I[i] + shObjW10.I[i] + shObjW11.I[i]
             + shObjW12.I[i] + shObjW13.I[i] + shObjW14.I[i] + shObjW15.I[i] +
            shObjW16.I[i] + shObjW17.I[i] + shObjW18.I[i] + shObjW19.I[i] + shObjW20.I[
            i] + shObjW21.I[i] + shObjW22.I[i] + shObjW23.I[i])/nS);

          FinalSVF3[i] =max(0,1 - nS + (shObjW1.SVF3[i] +
            shObjW2.SVF3[i] + shObjW3.SVF3[i] + shObjW4.SVF3[i] + shObjW5.SVF3[i] + shObjW6.SVF3[i] +
            shObjW7.SVF3[i] + shObjW8.SVF3[i] + shObjW9.SVF3[i] + shObjW10.SVF3[i] + shObjW11.SVF3[i]
             + shObjW12.SVF3[i] + shObjW13.SVF3[i] + shObjW14.SVF3[i] + shObjW15.SVF3[i] +
            shObjW16.SVF3[i] + shObjW17.SVF3[i] + shObjW18.SVF3[i] + shObjW19.SVF3[i] + shObjW20.SVF3[
            i] + shObjW21.SVF3[i] + shObjW22.SVF3[i] + shObjW23.SVF3[i]));
        end for;

        for i in 1:10 loop
          FinalSVF[i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalSVF2[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[i] = FinalSVF2[i];
        end for;

        for i in 21:30 loop
          FinalSVF[i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalSVF2[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[i] = FinalSVF2[i];
        end for;

        for i in 41:50 loop
          FinalSVF[i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalSVF2[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[i] = FinalSVF2[i];
        end for;

        for i in 11:20 loop
          FinalSVF[31-i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalSVF2[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[31-i] = FinalSVF2[i];
        end for;

        for i in 31:40 loop
          FinalSVF[71-i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalSVF2[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[71-i] = FinalSVF2[i];
        end for;

        for i in 51:60 loop
          FinalSVF[111-i] = (FinalSVF1[i]*shObjW1.SkyDifIso + FinalI[i]*shObjW1.SkyDifCir + FinalSVF3[i]*shObjW1.SkyDifHor)
            /(shObjW1.SkyDifIso+shObjW1.SkyDifCir+shObjW1.SkyDifHor);
          FinalI[111-i] = FinalSVF2[i];
        end for;

        facdif = sum(FinalSVF)/(n*k);
        facdir = sum(FinalI)/(n*k);

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                  -100},{100,100}})),                                  Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-140,-100},{100,100}}), graphics={
              Line(
                points={{0,0},{0,100}},
                color={238,46,47},
                thickness=0.5),
              Text(
                extent={{-140,6},{-116,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="WEST
[+90 deg]"),  Text(
                extent={{-18,112},{28,82}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="NORTH [+/-180 deg]"),
              Text(
                extent={{78,6},{100,-8}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="EAST
[-90 deg]"),  Text(
                extent={{-14,-86},{22,-108}},
                lineColor={0,0,0},
                lineThickness=0.5,
                textString="SOUTH [0 deg]")}),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600,
            __Dymola_Algorithm="Euler"),
          __Dymola_experimentSetupOutput(events=false));
      end MultiFacW60;
    end IrrModel60;

  end OfficeZave250520;

  annotation (uses(Modelica(version="3.2.3"), IDEAS(version="2.1.0")));
end BEM;

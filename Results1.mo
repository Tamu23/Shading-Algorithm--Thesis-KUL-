within ;
package Results1 "Irr models for Results1"

  model ShObj "single shading facade model trimmed"

    extends Modelica.Blocks.Icons.Block;

    parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
    parameter Modelica.SIunits.Length TgtFace_len(min=0)=3 "Length of facade";
    parameter Modelica.SIunits.Length TgtFace_H(min=0)=3 "Height of facade";
    parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
    /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
    parameter Real res(min=0)=100 "Resolution of Descritization; =1 for 1cm";
    parameter Modelica.SIunits.Length Shade_Face_len(min=0)=5 "Length of facade";
    parameter Modelica.SIunits.Length Shade_Face_H(min=0)=5 "Height of facade";
    parameter Modelica.SIunits.Angle Shade_Face_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
    /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
    parameter Modelica.SIunits.Length Shade_Face_D(min=0)=5 "Distance between target and shadow facade origins [See Figure] ";
    /* Ensure shading facade does not cross North Axis */
    parameter Modelica.SIunits.Angle Shade_Face_theta(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(-30)
    "Angular orientation of line connecting target and shadow facade start-points [See Figure]";

    Real SkyDifSH(min=0,unit="W/m2");
    Real DirSH(min=0,unit="W/m2");
    Real Glo(min=0,unit="W/m2");

    inner IDEAS.BoundaryConditions.SimInfoManager sim
      annotation (Placement(transformation(extent={{-96,76},{-76,96}})));
  //protected
    IRR.Irr.Components_Irr.DiffusePerez HDifTil(
      til=1.5707963267949,
      lat=lat,
      azi=TgtFace_azi,
      outSkyCon=true,
      outGroCon=true)
      annotation (Placement(transformation(extent={{-44,40},{-24,60}})));
    parameter Integer n = integer(floor((TgtFace_len*100/res) + 0.5)) + 1 "No. of discrete points in XY plane";
    parameter Integer k = integer(floor((TgtFace_H*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis";
    Modelica.SIunits.Length x0y0[n,2];
    Modelica.SIunits.Angle azmid[n];
    Modelica.SIunits.Length d[n];
    Modelica.SIunits.Length H_sh[n];
    //Modelica.SIunits.SolidAngle SAngle[n*k];
    constant Modelica.SIunits.SolidAngle pii = Modelica.Constants.pi;
    Modelica.SIunits.Angle Sun_Az1[6] = sim.radSol.solAzi.solAzi, Sun_Zen1[6] = sim.radSol.angZen;
    Real SVF1[n*k];
    Real SVF3[n*k];
    Real SkyDifIso(min=0,unit="W/m2");
    Real SkyDifCir(min=0,unit="W/m2");
    Real SkyDifHor(min=0,unit="W/m2");
    Real fac1, fac2, fac3, fac;
    Real I[n*k];
    Real SVF[n*k];

    IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
      til=1.5707963267949,
      lat=lat,
      azi=TgtFace_azi)
      annotation (Placement(transformation(extent={{-42,74},{-22,94}})));
  equation

    SkyDifIso = HDifTil.HDifTil.HDifHor*(0.5*(1 - HDifTil.HDifTil.briCof1));
    SkyDifCir = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof1*HDifTil.HDifTil.a/HDifTil.HDifTil.b;
    SkyDifHor = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof2;

    for i in 1:n loop
      x0y0[i,1] = Shade_Face_D*Modelica.Math.cos(-Modelica.Constants.pi/2+Shade_Face_theta)
      - (0 + ((i-1)*res*Modelica.Math.cos(TgtFace_azi)))/100;
      x0y0[i,2] = Shade_Face_D*Modelica.Math.sin(-Modelica.Constants.pi/2+Shade_Face_theta)
      - (0 + ((i-1)*res*Modelica.Math.sin(TgtFace_azi)))/100;
      //az0[i] =IRR.Irr_Dif3.Components_Dif3.Functions_Dif3.tanInv(x0y0[i,1], x0y0[i,2]);
      azmid[i] = IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + (
        Shade_Face_len/2)*Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + (
        Shade_Face_len/2)*Modelica.Math.sin(Shade_Face_azi));

      d[i] = IRR.Irr.Components_Irr.Functions_Irr.dist(
        azmid[i],
        Shade_Face_azi,
        IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
        TgtFace_azi,
        IRR.Irr.Components_Irr.Functions_Irr.Di(
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          Shade_Face_D,
          Shade_Face_theta,
          (i-1)*res/100,
          TgtFace_azi));

      H_sh[i] = IRR.Irr.Components_Irr.Functions_Irr.H_SH(
        IRR.Irr.Components_Irr.Functions_Irr.distDir(
          Sun_Az1[1],
          Shade_Face_azi,
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + Shade_Face_len*
            Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + Shade_Face_len*
            Modelica.Math.sin(Shade_Face_azi)),
          TgtFace_azi,
          IRR.Irr.Components_Irr.Functions_Irr.Di(
            IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
            Shade_Face_D,
            Shade_Face_theta,
            (i-1)*res/100,
            TgtFace_azi)),
        Shade_Face_H,
        Sun_Zen1[1]);

      for j in 1:k loop
        //SAngle[(i-1)*k+j] = Functions_Dif.SolidAngle(d[i], (j-1)*res/100, azmid[i], Shade_Face_azi, Shade_Face_len, Shade_Face_H);
        SVF1[(i - 1)*k + j] = (pii -
          IRR.Irr.Components_Irr.Functions_Irr.SolidAngle(
          d[i],
          (j - 1)*res/100,
          azmid[i],
          Shade_Face_azi,
          Shade_Face_len,
          Shade_Face_H))/pii;

        I[(i - 1)*k + j] = IRR.Irr.Components_Irr.Functions_Irr.Frac(H_sh[i], (j -
          1)*res/100);

        SVF3[(i-1)*k+j] = IRR.Irr.Components_Irr.Functions_Irr.SVF3(x0y0[i,1], x0y0[i,2], Shade_Face_len,
          Shade_Face_azi, Shade_Face_H, (j-1)*res/100, SkyDifHor);

        SVF[(i-1)*k+j] = (SVF1[(i-1)*k+j]*SkyDifIso + I[(i-1)*k+j]*SkyDifCir + SVF3[(i-1)*k+j]*SkyDifHor)/HDifTil.HDifTil.HSkyDifTil;

      end for;

    end for;

    fac1 = sum(SVF1)/(n*k);
    fac3 = sum(SVF3)/(n*k);
    fac2 = sum(I)/(n*k);
    SkyDifSH = fac2*SkyDifCir + fac1*SkyDifIso + fac3*SkyDifHor;
    fac = SkyDifSH/HDifTil.HSkyDifTil;
    DirSH = fac2*HDirTil.H;
    Glo = HDirTil.H + HDifTil.HDifTil.HSkyDifTil + HDifTil.HDifTil.HGroDifTil;


    connect(HDirTil.weaBus, sim.weaDatBus) annotation (Line(
        points={{-42,84},{-60,84},{-60,86},{-76.1,86}},
        color={255,204,51},
        thickness=0.5));
    connect(HDifTil.weaBus, sim.weaDatBus) annotation (Line(
        points={{-44,50},{-60,50},{-60,86},{-76.1,86}},
        color={255,204,51},
        thickness=0.5));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=31536000,
        Interval=600,
        __Dymola_fixedstepsize=600,
        __Dymola_Algorithm="Euler"));
  end ShObj;

  model ShObj1 "single shading facade model trimmed"

    extends Modelica.Blocks.Icons.Block;

    parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
    parameter Modelica.SIunits.Length TgtFace_len(min=0)=3 "Length of facade";
    parameter Modelica.SIunits.Length TgtFace_H(min=0)=3 "Height of facade";
    parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
    /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
    parameter Real res(min=0)=100 "Resolution of Descritization; =1 for 1cm";
    parameter Modelica.SIunits.Length Shade_Face_len(min=0)=5 "Length of facade";
    parameter Modelica.SIunits.Length Shade_Face_H(min=0)=5 "Height of facade";
    parameter Modelica.SIunits.Angle Shade_Face_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
    /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
    parameter Modelica.SIunits.Length Shade_Face_D(min=0)=5 "Distance between target and shadow facade origins [See Figure] ";
    /* Ensure shading facade does not cross North Axis */
    parameter Modelica.SIunits.Angle Shade_Face_theta(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(-30)
    "Angular orientation of line connecting target and shadow facade start-points [See Figure]";

    Real SkyDifSH(min=0,unit="W/m2");
    Real DirSH(min=0,unit="W/m2");

  //protected
    IRR.Irr.Components_Irr.DiffusePerez HDifTil(
      til=1.5707963267949,
      lat=lat,
      azi=TgtFace_azi,
      outSkyCon=true,
      outGroCon=true)
      annotation (Placement(transformation(extent={{-66,54},{-46,74}})));
    parameter Integer n = integer(floor((TgtFace_len*100/res) + 0.5)) + 1 "No. of discrete points in XY plane";
    parameter Integer k = integer(floor((TgtFace_H*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis";
    Modelica.SIunits.Length x0y0[n,2];
    Modelica.SIunits.Angle azmid[n];
    Modelica.SIunits.Length d[n];
    Modelica.SIunits.Length H_sh[n];
    //Modelica.SIunits.SolidAngle SAngle[n*k];
    constant Modelica.SIunits.SolidAngle pii = Modelica.Constants.pi;
    Modelica.SIunits.Angle Sun_Az1[6] = sim.radSol.solAzi.solAzi, Sun_Zen1[6] = sim.radSol.angZen;
    Real SVF1[n*k];
    Real SVF3[n*k];
    Real SkyDifIso(min=0,unit="W/m2");
    Real SkyDifCir(min=0,unit="W/m2");
    Real SkyDifHor(min=0,unit="W/m2");
    Real fac1, fac2, fac3, fac;
    Real I[n*k];
    Real SVF[n*k];

    IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
      til=1.5707963267949,
      lat=lat,
      azi=TgtFace_azi)
      annotation (Placement(transformation(extent={{-34,72},{-14,92}})));
    inner IDEAS.BoundaryConditions.SimInfoManager sim
      annotation (Placement(transformation(extent={{-100,76},{-80,96}})));
  equation

    SkyDifIso = HDifTil.HDifTil.HDifHor*(0.5*(1 - HDifTil.HDifTil.briCof1));
    SkyDifCir = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof1*HDifTil.HDifTil.a/HDifTil.HDifTil.b;
    SkyDifHor = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof2;

    for i in 1:n loop
      x0y0[i,1] = Shade_Face_D*Modelica.Math.cos(-Modelica.Constants.pi/2+Shade_Face_theta)
      - (0 + ((i-1)*res*Modelica.Math.cos(TgtFace_azi)))/100;
      x0y0[i,2] = Shade_Face_D*Modelica.Math.sin(-Modelica.Constants.pi/2+Shade_Face_theta)
      - (0 + ((i-1)*res*Modelica.Math.sin(TgtFace_azi)))/100;
      //az0[i] =IRR.Irr_Dif3.Components_Dif3.Functions_Dif3.tanInv(x0y0[i,1], x0y0[i,2]);
      azmid[i] = IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + (
        Shade_Face_len/2)*Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + (
        Shade_Face_len/2)*Modelica.Math.sin(Shade_Face_azi));

      d[i] = IRR.Irr.Components_Irr.Functions_Irr.dist(
        azmid[i],
        Shade_Face_azi,
        IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
        TgtFace_azi,
        IRR.Irr.Components_Irr.Functions_Irr.Di(
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          Shade_Face_D,
          Shade_Face_theta,
          (i-1)*res/100,
          TgtFace_azi));

      H_sh[i] = IRR.Irr.Components_Irr.Functions_Irr.H_SH(
        IRR.Irr.Components_Irr.Functions_Irr.distDir(
          Sun_Az1[1],
          Shade_Face_azi,
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + Shade_Face_len*
            Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + Shade_Face_len*
            Modelica.Math.sin(Shade_Face_azi)),
          TgtFace_azi,
          IRR.Irr.Components_Irr.Functions_Irr.Di(
            IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
            Shade_Face_D,
            Shade_Face_theta,
            (i-1)*res/100,
            TgtFace_azi)),
        Shade_Face_H,
        Sun_Zen1[1]);

      for j in 1:k loop
        //SAngle[(i-1)*k+j] = Functions_Dif.SolidAngle(d[i], (j-1)*res/100, azmid[i], Shade_Face_azi, Shade_Face_len, Shade_Face_H);
        SVF1[(i - 1)*k + j] = (pii -
          IRR.Irr.Components_Irr.Functions_Irr.SolidAngle1(
          d[i],
          (j - 1)*res/100,
          azmid[i],
          Shade_Face_azi,
          Shade_Face_len,
          Shade_Face_H))/pii;

        I[(i - 1)*k + j] = IRR.Irr.Components_Irr.Functions_Irr.Frac(H_sh[i], (j -
          1)*res/100);

        SVF3[(i-1)*k+j] = IRR.Irr.Components_Irr.Functions_Irr.SVF3(x0y0[i,1], x0y0[i,2], Shade_Face_len,
          Shade_Face_azi, Shade_Face_H, (j-1)*res/100, SkyDifHor);

        SVF[(i-1)*k+j] = (SVF1[(i-1)*k+j]*SkyDifIso + I[(i-1)*k+j]*SkyDifCir + SVF3[(i-1)*k+j]*SkyDifHor)/HDifTil.HDifTil.HSkyDifTil;

      end for;

    end for;

    fac1 = sum(SVF1)/(n*k);
    fac3 = sum(SVF3)/(n*k);
    fac2 = sum(I)/(n*k);
    SkyDifSH = fac2*SkyDifCir + fac1*SkyDifIso + fac3*SkyDifHor;
    fac = SkyDifSH/HDifTil.HSkyDifTil;
    DirSH = fac2*HDirTil.H;

    connect(sim.weaDatBus, HDirTil.weaBus) annotation (Line(
        points={{-80.1,86},{-58,86},{-58,82},{-34,82}},
        color={255,204,51},
        thickness=0.5));
    connect(sim.weaDatBus, HDifTil.weaBus) annotation (Line(
        points={{-80.1,86},{-76,86},{-76,64},{-66,64}},
        color={255,204,51},
        thickness=0.5));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=31536000,
        Interval=600,
        __Dymola_fixedstepsize=600,
        __Dymola_Algorithm="Euler"));
  end ShObj1;

  block DitTil "Direct solar irradiation on a tilted surface"
    extends
      IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.PartialSolarIrradiation;

    parameter Modelica.SIunits.Angle lat "Latitude";
    parameter Modelica.SIunits.Angle azi "Surface azimuth";

  protected
    IDEAS.BoundaryConditions.SolarGeometry.IncidenceAngle incAng(
      final azi=azi,
      final til=til,
      final lat=lat) "Incidence angle"
      annotation (Placement(transformation(extent={{-50,-30},{-30,-10}})));
    IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.DirectTiltedSurface
      HDirTil "Direct irradition on tilted surface"
      annotation (Placement(transformation(extent={{0,-20},{40,20}})));

  equation
    connect(incAng.y, HDirTil.incAng) annotation (Line(
        points={{-29,-20},{-12,-20},{-12,-12},{-4,-12}},
        color={0,0,127}));

    connect(weaBus.HDirNor, HDirTil.HDirNor) annotation (Line(
        points={{-100,5.55112e-16},{-80,5.55112e-16},{-80,12},{-4,12}},
        color={255,204,51},
        thickness=0.5), Text(
        textString="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(HDirTil.HDirTil, H) annotation (Line(
        points={{42,1.22125e-15},{72,1.22125e-15},{72,5.55112e-16},{110,
            5.55112e-16}},
        color={0,0,127}));

    connect(weaBus, incAng.weaBus) annotation (Line(
        points={{-100,5.55112e-16},{-80,5.55112e-16},{-80,-20},{-50,-20}},
        color={255,204,51},
        thickness=0.5), Text(
        textString="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    annotation (
      defaultComponentName="HDirTil",
      Documentation(info="<html>
<p>
This component computes the direct solar irradiation on a tilted surface.
For a definition of the parameters, see the
<a href=\"modelica://IDEAS.BoundaryConditions.UsersGuide\">User's Guide</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
April 21, 2016, by Michael Wetter:<br/>
Removed duplicate instance <code>weaBus</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/461\">
https://github.com/ibpsa/modelica-ibpsa/issues/461</a>.
</li>
<li>
December 12, 2010, by Michael Wetter:<br/>
Added incidence angle as output as this is needed for the room model.
</li>
<li>
May 24, 2010, by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
              100}}), graphics={Text(
            extent={{-150,110},{150,150}},
            textString="%name",
            lineColor={0,0,255})}));
  end DitTil;
  annotation (uses(Modelica(version="3.2.3"), IDEAS(version="2.1.0")));
end Results1;

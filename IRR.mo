within ;
package IRR

  package Irr "With seperate SVFs for all 3 SkyDif components"
    package Components_Irr
      block DiffusePerez
        "Hemispherical diffuse irradiation on a tilted surface using Perez's anisotropic sky model"
        extends
          IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.PartialSolarIrradiation;

        parameter Real rho(min=0, max=1, final unit="1")=0.2 "Ground reflectance";
        parameter Modelica.SIunits.Angle lat "Latitude";
        parameter Modelica.SIunits.Angle azi "Surface azimuth";
        parameter Boolean outSkyCon=false
          "Output contribution of diffuse irradiation from sky";
        parameter Boolean outGroCon=false
          "Output contribution of diffuse irradiation from ground";

        Modelica.Blocks.Math.Add add "Block to add radiations"
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        Modelica.Blocks.Interfaces.RealOutput HSkyDifTil if outSkyCon
          "Hemispherical diffuse solar irradiation on a tilted surface from the sky"
          annotation (Placement(transformation(extent={{100,50},{120,70}})));
        Modelica.Blocks.Interfaces.RealOutput HGroDifTil if outGroCon
          "Hemispherical diffuse solar irradiation on a tilted surface from the ground"
          annotation (Placement(transformation(extent={{100,-70},{120,-50}})));

      //protected
        IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.DiffusePerez HDifTil(final til=
             til, final rho=rho) "Diffuse irradiation on tilted surface"
          annotation (Placement(transformation(extent={{0,-21},{42,21}})));
        IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.SkyClearness skyCle
          "Sky clearness"
          annotation (Placement(transformation(extent={{-62,16},{-54,24}})));
        IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.BrighteningCoefficient briCoe
          "Brightening coefficient"
          annotation (Placement(transformation(extent={{-40,-34},{-32,-26}})));
        IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.RelativeAirMass relAirMas
          "Relative air mass"
          annotation (Placement(transformation(extent={{-80,-44},{-72,-36}})));
        IDEAS.BoundaryConditions.SolarIrradiation.BaseClasses.SkyBrightness skyBri
          "Sky brightness"
          annotation (Placement(transformation(extent={{-60,-54},{-52,-46}})));
        IDEAS.BoundaryConditions.SolarGeometry.IncidenceAngle incAng(
          final lat=lat,
          final azi=azi,
          final til=til) "Incidence angle"
          annotation (Placement(transformation(extent={{-86,-96},{-76,-86}})));

      equation


        connect(relAirMas.relAirMas, skyBri.relAirMas) annotation (Line(
            points={{-71.6,-40},{-66,-40},{-66,-48.4},{-60.8,-48.4}},
            color={0,0,127}));
        connect(skyBri.skyBri, briCoe.skyBri) annotation (Line(
            points={{-51.6,-50},{-46,-50},{-46,-30},{-40.8,-30}},
            color={0,0,127}));
        connect(skyCle.skyCle, briCoe.skyCle) annotation (Line(
            points={{-53.6,20},{-46,20},{-46,-27.6},{-40.8,-27.6}},
            color={0,0,127}));
        connect(incAng.y, HDifTil.incAng) annotation (Line(
            points={{-75.5,-91},{-16,-91},{-16,-16},{-4.2,-16},{-4.2,-14.7}},
            color={0,0,127}));
        connect(weaBus.solZen, skyCle.zen) annotation (Line(
            points={{-100,5.55112e-16},{-86,5.55112e-16},{-86,17.6},{-62.8,17.6}},
            color={0,0,127}));
        connect(weaBus.solZen, relAirMas.zen) annotation (Line(
            points={{-100,5.55112e-16},{-86,5.55112e-16},{-86,-40},{-80.8,-40}},
            color={0,0,127}));
        connect(weaBus.solZen, briCoe.zen) annotation (Line(
            points={{-100,5.55112e-16},{-86,5.55112e-16},{-86,-20},{-66,-20},{-66,-32},
                {-40.8,-32},{-40.8,-32.4}},
            color={0,0,127}));
        connect(weaBus.HGloHor, skyCle.HGloHor) annotation (Line(
            points={{-100,5.55112e-16},{-92,5.55112e-16},{-92,22.4},{-62.8,22.4}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(weaBus.HDifHor, skyCle.HDifHor) annotation (Line(
            points={{-100,5.55112e-16},{-92,5.55112e-16},{-92,20},{-62.8,20}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(weaBus.HDifHor, skyBri.HDifHor) annotation (Line(
            points={{-100,5.55112e-16},{-92,5.55112e-16},{-92,-51.6},{-60.8,-51.6}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(weaBus.HGloHor, HDifTil.HGloHor) annotation (Line(
            points={{-100,5.55112e-16},{-70,0},{-38,0},{-38,16.8},{-4.2,16.8}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(weaBus.HDifHor, HDifTil.HDifHor) annotation (Line(
            points={{-100,5.55112e-16},{-38,5.55112e-16},{-38,10},{-4.2,10},{-4.2,
                10.5}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));

        connect(briCoe.F2, HDifTil.briCof2) annotation (Line(
            points={{-31.6,-31.6},{-24,-31.6},{-24,-2.1},{-4.2,-2.1}},
            color={0,0,127}));
        connect(briCoe.F1, HDifTil.briCof1) annotation (Line(
            points={{-31.6,-28.4},{-28,-28.4},{-28,4.2},{-4.2,4.2}},
            color={0,0,127}));
        connect(weaBus, incAng.weaBus) annotation (Line(
            points={{-100,5.55112e-16},{-92,5.55112e-16},{-92,-91},{-86,-91}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(weaBus.solZen, HDifTil.zen) annotation (Line(
            points={{-100,5.55112e-16},{-86,5.55112e-16},{-86,-58},{-20,-58},{-20,
                -8.4},{-4.2,-8.4}},
            color={255,204,51},
            thickness=0.5), Text(
            textString="%first",
            index=-1,
            extent={{-6,3},{-6,3}}));
        connect(HDifTil.HSkyDifTil, add.u1) annotation (Line(
            points={{44.1,8.4},{52,8.4},{52,6},{58,6}},
            color={0,0,127}));
        connect(HDifTil.HGroDifTil, add.u2) annotation (Line(
            points={{44.1,-8.4},{52,-8.4},{52,-6},{58,-6}},
            color={0,0,127}));
        connect(add.y, H) annotation (Line(
            points={{81,6.10623e-16},{90.5,6.10623e-16},{90.5,5.55112e-16},{110,
                5.55112e-16}},
            color={0,0,127}));

        connect(HDifTil.HSkyDifTil, HSkyDifTil) annotation (Line(
            points={{44.1,8.4},{52,8.4},{52,60},{110,60}},
            color={0,0,127}));
        connect(HDifTil.HGroDifTil, HGroDifTil) annotation (Line(
            points={{44.1,-8.4},{52,-8.4},{52,-60},{110,-60}},
            color={0,0,127}));
        annotation (
          defaultComponentName="HDifTil",
          Documentation(info="<html>
<p>
This component computes the hemispherical diffuse irradiation on a tilted surface using an anisotropic
sky model proposed by Perez.
For a definition of the parameters, see the
<a href=\"modelica://IDEAS.BoundaryConditions.UsersGuide\">User's Guide</a>.
</p>
<h4>References</h4>
<ul>
<li>
P. Ineichen, R. Perez and R. Seals (1987).
<i>The Importance of Correct Albedo Determination for Adequately Modeling Energy Received by Tilted Surface</i>,
Solar Energy, 39(4): 301-305.
</li>
<li>
R. Perez, R. Seals, P. Ineichen, R. Stewart and D. Menicucci (1987).
<i>A New Simplified Version of the Perez Diffuse Irradiance Model for Tilted Surface</i>,
Solar Energy, 39(3): 221-231.
</li>
<li>
R. Perez, P. Ineichen, R. Seals, J. Michalsky and R. Stewart (1990).
<i>Modeling Dyalight Availability and Irradiance Componets From Direct and Global Irradiance</i>,
Solar Energy, 44(5):271-289.
</li>
</ul>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2015, by Michael Wetter:<br/>
Added <code>min</code>, <code>max</code> and <code>unit</code>
attributes for <code>rho</code>.
</li>
<li>
June 6, 2012, by Wangda Zuo:<br/>
Added contributions from sky and ground that were separated in base class.
</li>
<li>
February 25, 2012, by Michael Wetter:<br/>
Changed component to get zenith angle from weather bus.
</li>
<li>
May 24, 2010, by Wangda Zuo:<br/>
First implementation.
</li>
</ul>
</html>"),Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}), graphics={Text(
                extent={{-150,110},{150,150}},
                textString="%name",
                lineColor={0,0,255})}));
      end DiffusePerez;

      model SimInfoManager
        "Simulation information manager for handling time and climate data required in each for simulation."
        extends IDEAS.BoundaryConditions.Interfaces.PartialSimInfoManager;

      protected
        Modelica.Blocks.Routing.RealPassThrough HDirNorData;
        Modelica.Blocks.Routing.RealPassThrough HGloHorData;
        Modelica.Blocks.Routing.RealPassThrough HDiffHorData;
        Modelica.Blocks.Routing.RealPassThrough TDryBulData;
        Modelica.Blocks.Routing.RealPassThrough relHumData;
        Modelica.Blocks.Routing.RealPassThrough TDewPoiData;
        Modelica.Blocks.Routing.RealPassThrough nOpaData;
        Modelica.Blocks.Routing.RealPassThrough winSpeData;
        Modelica.Blocks.Routing.RealPassThrough winDirData;
        Modelica.Blocks.Routing.RealPassThrough TBlaSkyData;
      equation
        Te = TDryBul.y;
        TeAv = Te;
        Tground=TdesGround;
        relHum = phiEnv.y;
        TDewPoi = TDewPoiData.y;
        Tsky = TBlaSkyData.y;
        Va = winSpeData.y;
        Vdir = winDirData.y;

        connect(HDirNorData.u, weaDatBus.HDirNor);
        connect(HGloHorData.u, weaDatBus.HGloHor);
        connect(HDiffHorData.u, weaDatBus.HDifHor);
        connect(TDryBulData.u, weaDatBus.TDryBul);
        connect(relHumData.u, weaDatBus.relHum);
        connect(TDewPoiData.u, weaDatBus.TDewPoi);
        connect(nOpaData.u, weaDatBus.nOpa);
        connect(winSpeData.u, weaDatBus.winSpe);
        connect(winDirData.u, weaDatBus.winDir);
        connect(TBlaSkyData.u, weaDatBus.TBlaSky);
        annotation (
          defaultComponentName="sim",
          defaultComponentPrefixes="inner",
          missingInnerMessage=
              "Your model is using an outer \"sim\" component. An inner \"sim\" component is not defined. For simulation drag IDEAS.BoundaryConditions.SimInfoManager into your model.",
          Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
               graphics={
              Bitmap(extent={{22,-8},{20,-8}}, fileName="")}),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,160}})),
          Documentation(info="<html>
<p>
The SimInfoManager manages all simulation information. 
It loads TMY3 weather data files and applies transformations 
for computing the solar irradiance on the zone surfaces. 
</p>
<h4>Typical use and important parameters</h4>
<ul>
<li>
Parameters <code>filNam</code> and <code>filDir</code> can be used to set the path to the TMY3 weather file.
This file should include the latitude, longitude and time zone corresponding to the weather file.
See the included weather files for the correct format.
</li>
</ul>
<h4>Options</h4>
<ul>
<li>
IDEAS contains an efficient implementation for computing the solar 
incidence angles on surfaces that are part of large building models.
When a model has many parallel surfaces the default implementation computes
the solar irradiance separately for each of these surfaces, 
while the result for all of them should be the same.
The SimInfoManager computes five default orientations (azimuth angels): 
south, west, east, north and horizontal.
Whenever a surface needs the solar incidence angels for one of these orientations
these precomputed values will be used.
The default orientations can be changed using parameters 
<code>incAndAziInBus</code>.
<code>incAndAziInBus</code> determines for which inclination and azimuth the solar radiation is pre-computed.
</li>
<li>Conservation of energy within the building can be checked by setting <code>computeConservationOfEnergy=true</code>.
Conservation of energy is checked by computing the internal energy for 
all components that are within \"the system\" and by adding to this the 
integral of all heat flows entering/leaving the system.
There are two options for choosing the extent of the system based 
on parameter <code>openSystemConservationOfEnergy</code>. 
Either conservation of energy for a closed system is computed, 
or it is computed for an open system. <br/>
When choosing the closed system the conservation of energy 
check should always work when using IDEAS as intended. 
In this case conservation of energy is only checked for all components in the <code>Buildings</code> package. 
I.e. all heat flows at embedded ports <code>port_emb</code> of walls, 
fluid ports of the zones, <code>zone.gainCon</code> and <code>zone.gainRad</code> are 
considered to be a heat gain to the system and every other component 
is considered to be outside of the system for which conservation of energy is checked. <br/>
When computing an open system by setting <code>openSystemConservationOfEnergy=true</code> 
these heat flow rates are not taken into account because they are assumed 
to flow between components that are both within the bounds of the system.
The user then needs to choose how large the system is and he should make sure that
all heat flow rates entering the system are added to <code>sim.Qgai.Q_flow</code> and 
that all internal energy of the system is added to <code>sim.E.E</code>.
</li>
<li>
The default latitude and longitude, which are read by the TMY3 reader, can be overwritten. 
This should only be done if a custom weather data reader instead 
of the TMY3 weather data reader is used.
</li>
</ul>
<h4>TMY3 weather data files</h4>
IDEAS uses TMY3 input files. For detailed documentation see 
<a href=\"modelica://IDEAS.BoundaryConditions.WeatherData.ReaderTMY3\">IDEAS.BoundaryConditions.WeatherData.ReaderTMY3</a>.
</html>",       revisions="<html>
<ul>
<li>
November 28, 2019 by Ian Beausoleil-Morrison:<br/>
Make wind direction available on WeaBus.
See <a href=\"https://github.com/open-ideas/IDEAS/issues/1089\">
#1089</a>
</li>
<li>
January 21, 2019 by Filip Jorissen:<br/>
Improved documentation by adding weather data reader
reference and more TMY3 file examples.
This is for
<a href=\"https://github.com/open-ideas/IDEAS/issues/956\">#956</a>.
</li>
<li>
June 7, 2018 by Filip Jorissen:<br/>
Overwriting TSky, Va and Fc from the extends clause
such that they can be overwriten again in BESTEST SimInfoManager.
This is for
<a href=\"https://github.com/open-ideas/IDEAS/issues/838\">#838</a>.
</li>
<li>
June 14, 2015, Filip Jorissen:<br/>
Added documentation
</li>
</ul>
</html>"));
      end SimInfoManager;

      package Functions_Irr
        function delaz "Delta azimuth angle subtended by shading facade"
          input Modelica.SIunits.Length x0, y0;
          input Modelica.SIunits.Length lenS;
          input Modelica.SIunits.Angle aziS;
          output Modelica.SIunits.Angle deltaAZ;

        protected
          Modelica.SIunits.Angle az0 = tanInv(x0,y0);
          Modelica.SIunits.Angle az1 = tanInv(x0 + lenS*Modelica.Math.cos(aziS),
            y0 + lenS*Modelica.Math.sin(aziS));

        algorithm
          deltaAZ :=if az0 > az1 then az0 - az1 else az1 - az0;

        end delaz;

        function tanInv
          input Modelica.SIunits.Length x;
          input Modelica.SIunits.Length y;
          output Modelica.SIunits.Angle angle;

        algorithm
          if y>=0 then
            if x>0 then
              angle := Modelica.Math.atan(y/x) + Modelica.Constants.pi/2;
            elseif x<0 then
              angle := Modelica.Math.atan(y/x) - Modelica.Constants.pi/2;
            else
              angle := Modelica.Constants.pi;
            end if;
          else
            if x>0 then
              angle := Modelica.Math.atan(y/x) + Modelica.Constants.pi/2;
            elseif x<0 then
              angle := Modelica.Math.atan(y/x) - Modelica.Constants.pi/2;
            else
              angle := 0;
            end if;
          end if;

        end tanInv;

        function dist
          input Modelica.SIunits.Angle az_i;
          input Modelica.SIunits.Angle az_S;
          input Modelica.SIunits.Angle az_0;
          input Modelica.SIunits.Angle az_T;
          input Modelica.SIunits.Length D0;
          output Modelica.SIunits.Length d;

        protected
          Real coz = Modelica.Math.cos(az_T-az_i);

        algorithm

          if coz>0 then
            if Modelica.Math.cos(az_S)==0 then

              if Modelica.Math.sin(az_i)==0 then
                d := 0;
              else d := D0*Modelica.Math.sin(az_0)/Modelica.Math.sin(az_i);
              end if;

            elseif Modelica.Math.sin(az_S)==0 then

              if Modelica.Math.cos(az_i)==0 then
                d := 0;
              else d:= D0*Modelica.Math.cos(az_0)/Modelica.Math.cos(az_i);
              end if;

            else d :=D0*(Modelica.Math.cos(az_0) + Modelica.Math.sin(az_0)*Modelica.Math.tan(az_S))
            /(Modelica.Math.sin(az_i)*Modelica.Math.tan(az_S) + Modelica.Math.cos(az_i));
            end if;
          else d := 0;
          end if;

        end dist;

        function SolidAngle
          input Modelica.SIunits.Length d;
          input Modelica.SIunits.Length z_T;
          input Modelica.SIunits.Angle az_i;
          input Modelica.SIunits.Angle az_s;
          input Modelica.SIunits.Length L_s;
          input Modelica.SIunits.Length H_s;
          output Modelica.SIunits.SolidAngle SA;

        protected
          Modelica.SIunits.Length L1 = L_s*Modelica.Math.cos(az_i-az_s);
          Modelica.SIunits.Length L = if L1<0 then -L1 else L1;
          Real cozV = if L1<0 then -L1/L_s else L1/L_s;
          Modelica.SIunits.Length H = 2*(H_s-z_T);

        algorithm

          SA := if d>0 and H>0 then 0.5*4*Modelica.Math.asin((L*H)/(((L*L + 4*d*d)*(H*H + 4*d*d))^0.5)) else 0;

        end SolidAngle;

        function Di
          input Modelica.SIunits.Angle az0;
          input Modelica.SIunits.Length D0;
          input Modelica.SIunits.Angle th0;
          input Modelica.SIunits.Length I;
          input Modelica.SIunits.Angle azT;
          output Modelica.SIunits.Length Di;

        algorithm

          if Modelica.Math.cos(az0) == 0 then
            Di :=D0*Modelica.Math.sin(th0) - I*Modelica.Math.cos(
              azT);
          else Di :=(D0^2+I^2+2*D0*I*(Modelica.Math.sin(azT-th0)))^0.5;
          end if;

        end Di;

        function Frac
          input Modelica.SIunits.Length h_sh;
          input Modelica.SIunits.Length zT;
          output Real InOut;

        algorithm

          InOut := if zT > h_sh then 1 else 0;

        end Frac;

        function H_SH
          input Modelica.SIunits.Length d;
          input Modelica.SIunits.Length H;
          input Modelica.SIunits.Angle zen;
          output Modelica.SIunits.Length h_sh;

        algorithm
          if d>0 then
            if Modelica.Math.cos(zen)>0 and Modelica.Math.cos(zen)<1 then
              h_sh := max(-0.01,H - d/Modelica.Math.tan(zen));
            elseif Modelica.Math.cos(zen)==1 then
              h_sh := -0.01;
            else
              h_sh := H;
            end if;
          else
            h_sh := -0.01;
          end if;

        end H_SH;

        function distDir
          input Modelica.SIunits.Angle az_i;
          input Modelica.SIunits.Angle az_S;
          input Modelica.SIunits.Angle az_0;
          input Modelica.SIunits.Angle az_1;
          input Modelica.SIunits.Angle az_T;
          input Modelica.SIunits.Length D0;
          output Modelica.SIunits.Length d;

        protected
          Real coz = Modelica.Math.cos(az_T-az_i);
          Modelica.SIunits.Angle azA = min(az_0, az_1);
          Modelica.SIunits.Angle azB = max(az_0, az_1);

        algorithm
          if az_i<azA or az_i>azB then
            d := 0;
          else
            if coz>0 then
              if Modelica.Math.cos(az_S)==0 then

                if Modelica.Math.sin(az_i)==0 then
                  d := 0;
                else d := D0*Modelica.Math.sin(az_0)/Modelica.Math.sin(az_i);
                end if;

              elseif Modelica.Math.sin(az_S)==0 then

                if Modelica.Math.cos(az_i)==0 then
                  d := 0;
                else d:= D0*Modelica.Math.cos(az_0)/Modelica.Math.cos(az_i);
                end if;

              else d :=D0*(Modelica.Math.cos(az_0) + Modelica.Math.sin(az_0)*Modelica.Math.tan(az_S))
              /(Modelica.Math.sin(az_i)*Modelica.Math.tan(az_S) + Modelica.Math.cos(az_i));
              end if;
            else d := 0;
            end if;
          end if;

        end distDir;

        function FinalI
          input Real avgI;
          output Real finalI;

        algorithm

          if avgI<1 then
            finalI :=0;
          else finalI :=1;
          end if;

        end FinalI;

        function SVF3
          input Modelica.SIunits.Length x0;
          input Modelica.SIunits.Length y0;
          input Modelica.SIunits.Length lenS;
          input Modelica.SIunits.Angle aziS;
          input Modelica.SIunits.Length hS;
          input Modelica.SIunits.Length zT;
          input Real SDHor;
          output Real SVF3;

        protected
          Modelica.SIunits.Angle az0 = Irr.Components_Irr.Functions_Irr.tanInv(x0, y0);
          Modelica.SIunits.Angle az1 = Irr.Components_Irr.Functions_Irr.tanInv(x0+lenS*Modelica.Math.cos(aziS), y0+lenS*Modelica.Math.sin(aziS));
          Modelica.SIunits.Angle thetaH = if az0 > az1 then az0-az1 else az1-az0;
          Modelica.SIunits.Length H = hS-zT;

        algorithm
          SVF3 :=if H>0 then (Modelica.Constants.pi-thetaH)/Modelica.Constants.pi else 1;

        end SVF3;

        function SolidAngle1
          input Modelica.SIunits.Length R;
          input Modelica.SIunits.Length z_T;
          input Modelica.SIunits.Angle az_i;
          input Modelica.SIunits.Angle az_s;
          input Modelica.SIunits.Length B;
          input Modelica.SIunits.Length H_s;
          output Modelica.SIunits.SolidAngle SA;

        protected
          Modelica.SIunits.Length L1 = (H_s-z_T);
          Real cza = Modelica.Math.cos(Modelica.Constants.pi/2+az_i-az_s);
          Real sna = Modelica.Math.sin(Modelica.Constants.pi/2+az_i-az_s);
          Modelica.SIunits.Length L2 = -(H_s-z_T);
          Modelica.SIunits.Length B1 = B/2 + R*cza;
          Modelica.SIunits.Length B2 = B/2 - R*cza;
          Modelica.SIunits.Length H = R*sna;
          Modelica.SIunits.SolidAngle om11 = Modelica.Math.asin((L1*B1)/(((L1^2 + H^2)*(B1^2 + H^2))^0.5));
          Modelica.SIunits.SolidAngle om12 = Modelica.Math.asin((L1*B2)/(((L1^2 + H^2)*(B2^2 + H^2))^0.5));
          Modelica.SIunits.SolidAngle om21 = Modelica.Math.asin((L2*B1)/(((L2^2 + H^2)*(B1^2 + H^2))^0.5));
          Modelica.SIunits.SolidAngle om22 = Modelica.Math.asin((L2*B2)/(((L2^2 + H^2)*(B2^2 + H^2))^0.5));

        algorithm
          SA := if R>0 and L1>0 then 0.5*(om11+om12-om21-om22) else 0;

        end SolidAngle1;

        function SolidAngleAlt
          input Modelica.SIunits.Length d;
          input Modelica.SIunits.Length z_T;
          input Modelica.SIunits.Angle az_i;
          input Modelica.SIunits.Angle az_s;
          input Modelica.SIunits.Length L_s;
          input Modelica.SIunits.Length H_s;
          output Modelica.SIunits.SolidAngle SA;

        protected
          Modelica.SIunits.Length L1 = L_s*Modelica.Math.cos(az_i-az_s);
          Modelica.SIunits.Length L = if L1<0 then -L1 else L1;
          Real cozV = if L1<0 then -L1/L_s else L1/L_s;
          Modelica.SIunits.Length H = 2*(H_s-z_T);

        algorithm

          SA := if d>0 and H>0 then 0.5*4*Modelica.Math.asin((L*H)/(((L*L + 4*d*d)*(H*H + 4*d*d))^0.5)) else 0;

        end SolidAngleAlt;
      end Functions_Irr;

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
        Real SVF[n*k];
        Real I[n*k];

      protected
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

        IRR.Irr.Components_Irr.DiffusePerez HDifTil(
          til=1.5707963267949,
          lat=lat,
          azi=TgtFace_azi,
          outSkyCon=true,
          outGroCon=true)
          annotation (Placement(transformation(extent={{-66,54},{-46,74}})));

        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-98,80},{-78,100}})));
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

        connect(HDifTil.weaBus, sim.weaDatBus) annotation (Line(
            points={{-66,64},{-72,64},{-72,90},{-78.1,90}},
            color={255,204,51},
            thickness=0.5));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600.001,
            __Dymola_Algorithm="Euler"));
      end ShObj;

      model ShObjW "single shading facade model trimmed"

        extends Modelica.Blocks.Icons.Block;

        parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";

        parameter Modelica.SIunits.Length winW = 2 "Width of subsurface";
        parameter Modelica.SIunits.Length winH = 2 "Height of subsurface";
        parameter Modelica.SIunits.Length winl = 1 "Horizontal distance of subsurface edge from facade edge";
        parameter Modelica.SIunits.Length winz = 1 "Vertical distance of subsurface edge from ground";
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
        Real SVF[n*k];
        Real SVF1[n*k];
        Real SVF3[n*k];
        Real I[n*k];
        Real SkyDifIso(unit="W/m2");
        Real SkyDifCir(unit="W/m2");
        Real SkyDifHor(unit="W/m2");

      protected
        parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane";
        parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis";
        Modelica.SIunits.Length x0y0[n,2];
        Modelica.SIunits.Angle azmid[n];
        Modelica.SIunits.Length d[n];
        Modelica.SIunits.Length H_sh[n];
        //Modelica.SIunits.SolidAngle SAngle[n*k];
        constant Modelica.SIunits.SolidAngle pii = Modelica.Constants.pi;
        Modelica.SIunits.Angle Sun_Az1[6] = sim.radSol.solAzi.solAzi, Sun_Zen1[6] = sim.radSol.angZen;

        IRR.Irr.Components_Irr.DiffusePerez HDifTil(
          til=1.5707963267949,
          lat=lat,
          azi=TgtFace_azi,
          outSkyCon=true,
          outGroCon=true)
          annotation (Placement(transformation(extent={{-66,54},{-46,74}})));

        inner IDEAS.BoundaryConditions.SimInfoManager sim
          annotation (Placement(transformation(extent={{-98,76},{-78,96}})));
      equation

        SkyDifIso = HDifTil.HDifTil.HDifHor*(0.5*(1-HDifTil.HDifTil.briCof1));
        SkyDifCir = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof1*HDifTil.HDifTil.a/HDifTil.HDifTil.b;
        SkyDifHor = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof2;

        for i in 1:n loop
          x0y0[i,1] = Shade_Face_D*Modelica.Math.cos(-Modelica.Constants.pi/2+Shade_Face_theta)
          - (winl+(i-1)*res/100)*Modelica.Math.cos(TgtFace_azi);
          x0y0[i,2] = Shade_Face_D*Modelica.Math.sin(-Modelica.Constants.pi/2+Shade_Face_theta)
          - (winl+(i-1)*res/100)*Modelica.Math.sin(TgtFace_azi);
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
              winl+(i-1)*res/100,
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
                winl+(i-1)*res/100,
                TgtFace_azi)),
            Shade_Face_H,
            Sun_Zen1[1]);

          for j in 1:k loop
            //SAngle[(i-1)*k+j] = Functions_Dif.SolidAngle(d[i], (j-1)*res/100, azmid[i], Shade_Face_azi, Shade_Face_len, Shade_Face_H);
            SVF1[(i-1)*k+j] = (pii -
              IRR.Irr.Components_Irr.Functions_Irr.SolidAngle(
              d[i],
              winz+(j-1)*res/100,
              azmid[i],
              Shade_Face_azi,
              Shade_Face_len,
              Shade_Face_H))/pii;

            I[(i-1)*k+j] = IRR.Irr.Components_Irr.Functions_Irr.Frac(H_sh[i],
              winz+(j-1)*res/100);

            SVF3[(i-1)*k+j] = IRR.Irr.Components_Irr.Functions_Irr.SVF3(x0y0[i,1], x0y0[i,2], Shade_Face_len,
              Shade_Face_azi, Shade_Face_H, winz+(j-1)*res/100, SkyDifHor);

            SVF[(i-1)*k+j] = (SVF1[(i-1)*k+j]*SkyDifIso + I[(i-1)*k+j]*SkyDifCir + SVF3[(i-1)*k+j]*SkyDifHor)/HDifTil.HSkyDifTil;

          end for;

        end for;

        connect(HDifTil.weaBus, sim.weaDatBus) annotation (Line(
            points={{-66,64},{-74,64},{-74,86},{-78.1,86}},
            color={255,204,51},
            thickness=0.5));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            Interval=600,
            __Dymola_fixedstepsize=600,
            __Dymola_Algorithm="Euler"));
      end ShObjW;

    end Components_Irr;

    model MultiFac

      extends Modelica.Blocks.Icons.Block;

      parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
      parameter Modelica.SIunits.Length TgtFace_len(min=0)=3 "Length of facade";
      parameter Modelica.SIunits.Length TgtFace_H(min=0)=3 "Height of facade";
      parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Real res(min=0)=100 "Resolution of descritization; =1 for 1cm";
      constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
      parameter Integer nS(min=1)=2 "No. of shading objects";
      final parameter Integer n = integer(floor((TgtFace_len*100/res) + 0.5)) + 1 "No. of discrete points in XY plane" annotation(HideResult=true);
      final parameter Integer k = integer(floor((TgtFace_H*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis" annotation(HideResult=true);
      Real FinalSVF[n*k];
      Real facdif;
      //Real DifIrr_sh(min=0,unit="W/m2");
      Real FinalI[n*k];
      Real facdir;
      //Real DirIrr_sh(min=0,unit="W/m2");

      Components_Irr.ShObj ShObj1(
        lat=lat,
        TgtFace_len=TgtFace_len,
        TgtFace_H=TgtFace_H,
        TgtFace_azi=TgtFace_azi,
        res=res) annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
      Components_Irr.ShObj ShObj2(
        lat=lat,
        TgtFace_len=TgtFace_len,
        TgtFace_H=TgtFace_H,
        TgtFace_azi=TgtFace_azi,
        res=res,
        Shade_Face_azi=-1.5707963267949,
        Shade_Face_theta=-1.5707963267949)
        annotation (Placement(transformation(extent={{34,-20},{54,0}})));
      inner Components_Irr.SimInfoManager sim
        annotation (Placement(transformation(extent={{-138,78},{-118,98}})));
    equation

      for i in 1:n*k loop
        /*Add shading object "DifShObj1,DifShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
        FinalSVF[i] =1 - nS + (ShObj1.SVF[i] + ShObj2.SVF[i]);

        /*Add shading object "ShObj1,ShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
        FinalI[i] = Components_Irr.Functions_Irr.FinalI((ShObj1.I[i] + ShObj2.I[i])
        /nS);
      end for;

      facdif = sum(FinalSVF)/(n*k);
      //DifIrr_sh = facdif*HDifTil.HSkyDifTil + HDifTil.HGroDifTil;
      facdir = sum(FinalI)/(n*k);
      //DirIrr_sh = facdir*HDirTil.H;

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
[+90 deg]"),Text(
              extent={{-18,112},{28,82}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="NORTH [+/-180 deg]"),
            Text(
              extent={{78,6},{100,-8}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="EAST
[-90 deg]"),Text(
              extent={{-14,-86},{22,-108}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="SOUTH [0 deg]")}),
        experiment(
          StopTime=31536000,
          Interval=600,
          __Dymola_fixedstepsize=600,
          __Dymola_Algorithm="Euler"));
    end MultiFac;

    model MultiFacW

      extends Modelica.Blocks.Icons.Block;

      parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
      parameter Modelica.SIunits.Length winW = 3 "Width of subsurface";
      parameter Modelica.SIunits.Length winH = 3 "Height of subsurface";
      parameter Modelica.SIunits.Length winl = 0 "Horizontal distance of subsurface edge from facade edge";
      parameter Modelica.SIunits.Length winz = 0 "Vertical distance of subsurface edge from ground";
      parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Real res(min=0)=100 "Resolution of descritization; =1 for 1cm";
      constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
      parameter Integer nS(min=1)=2 "No. of shading objects";
      final parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane" annotation(HideResult=true);
      final parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis" annotation(HideResult=true);
      Real FinalSVF1[n*k];
      Real FinalSVF3[n*k];
      Real FinalSVF[n*k];
      Real facdif;
      //Real DifIrr_sh(min=0,unit="W/m2");
      Real FinalI[n*k];
      Real facdir;
      //Real DirIrr_sh(min=0,unit="W/m2");

      Components_Irr.ShObjW ShObjW1(
        lat=lat,
        winW=winW,
        winH=winH,
        winl=winl,
        winz=winz)
        annotation (Placement(transformation(extent={{-10,-56},{10,-36}})));
      Components_Irr.ShObjW ShObjW2(
        lat=lat,
        winW=winW,
        winH=winH,
        winl=winl,
        winz=winz,
        Shade_Face_azi=-1.5707963267949,
        Shade_Face_theta=-1.5707963267949)
        annotation (Placement(transformation(extent={{32,-20},{52,0}})));
      inner Components_Irr.SimInfoManager sim
        annotation (Placement(transformation(extent={{-138,78},{-118,98}})));
    equation

      for i in 1:n*k loop
        /*Add shading object "DifShObj1,DifShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
        FinalSVF1[i] =1 - nS + (ShObjW1.SVF1[i] + ShObjW2.SVF1[i]);

        FinalSVF3[i] =1 - nS + (ShObjW1.SVF3[i] + ShObjW2.SVF3[i]);

        /*Add shading object "ShObj1,ShObj2,3,.. and so on according to the number of shading objects in your simulation below:*/
        FinalI[i] = Components_Irr.Functions_Irr.FinalI((ShObjW1.I[i] + ShObjW2.I[i])
        /nS);

      FinalSVF[i] = (FinalSVF1[i]*ShObjW1.SkyDifIso + FinalI[i]*ShObjW1.SkyDifCir + FinalSVF3[i]*ShObjW1.SkyDifHor)
          /(ShObjW1.SkyDifIso+ShObjW1.SkyDifCir+ShObjW1.SkyDifHor);

      end for;

      facdif = sum(FinalSVF)/(n*k);
      //DifIrr_sh = facdif*HDifTil.HSkyDifTil + HDifTil.HGroDifTil;
      facdir = sum(FinalI)/(n*k);
      //DirIrr_sh = facdir*HDirTil.H;

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
[+90 deg]"),Text(
              extent={{-18,112},{28,82}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="NORTH [+/-180 deg]"),
            Text(
              extent={{78,6},{100,-8}},
              lineColor={0,0,0},
              lineThickness=0.5,
              textString="EAST
[-90 deg]"),Text(
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
  end Irr;

  package IrrAlt "Using better solidangle calculations"
    model ShObjWAlt "single shading facade model ALT"

      extends Modelica.Blocks.Icons.Block;

      parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";

      parameter Modelica.SIunits.Length winW = 3 "Width of subsurface";
      parameter Modelica.SIunits.Length winH = 3 "Height of subsurface";
      parameter Modelica.SIunits.Length winl = 0 "Horizontal distance of subsurface edge from facade edge";
      parameter Modelica.SIunits.Length winz = 0 "Vertical distance of subsurface edge from ground";
      parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Real res(min=0)=10 "Resolution of Descritization; =1 for 1cm";
      parameter Modelica.SIunits.Length Shade_Face_len(min=0)=5 "Length of facade";
      parameter Modelica.SIunits.Length Shade_Face_H(min=0)=5 "Height of facade";
      parameter Modelica.SIunits.Angle Shade_Face_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(0) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Modelica.SIunits.Length Shade_Face_D(min=0)=5 "Distance between target and shadow facade origins [See Figure] ";
      /* Ensure shading facade does not cross North Axis */
      parameter Modelica.SIunits.Angle Shade_Face_theta(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(-30)
      "Angular orientation of line connecting target and shadow facade start-points [See Figure]";
      Real SVF[n*k];
      Real I[n*k];

    //protected
      inner Irr.Components_Irr.SimInfoManager sim
        annotation (Placement(transformation(extent={{-98,78},{-78,98}})));
      parameter Integer n = integer(floor((winW*100/res) + 0.5)) + 1 "No. of discrete points in XY plane";
      parameter Integer k = integer(floor((winH*100/res) + 0.5)) + 1 "No. of discrete points along Z-axis";
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
      //Real SkyDifSH(min=0,unit="W/m2");
      //Real fac1, fac2, fac3;

      IRR.Irr.Components_Irr.DiffusePerez HDifTil(
        til=1.5707963267949,
        lat=lat,
        azi=TgtFace_azi,
        outSkyCon=true,
        outGroCon=true)
        annotation (Placement(transformation(extent={{-66,54},{-46,74}})));

    equation

      SkyDifIso = HDifTil.HDifTil.HDifHor*(0.5*(1 - HDifTil.HDifTil.briCof1));
      SkyDifCir = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof1*HDifTil.HDifTil.a/HDifTil.HDifTil.b;
      SkyDifHor = HDifTil.HDifTil.HDifHor*HDifTil.HDifTil.briCof2;

      for i in 1:n loop
        x0y0[i,1] = Shade_Face_D*Modelica.Math.cos(-Modelica.Constants.pi/2+Shade_Face_theta)
        - (winl + ((i-1)*res*Modelica.Math.cos(TgtFace_azi)))/100;
        x0y0[i,2] = Shade_Face_D*Modelica.Math.sin(-Modelica.Constants.pi/2+Shade_Face_theta)
        - (winl + ((i-1)*res*Modelica.Math.sin(TgtFace_azi)))/100;
        //az0[i] =IRR.Irr_Dif3.Components_Dif3.Functions_Dif3.tanInv(x0y0[i,1], x0y0[i,2]);
        azmid[i] = IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + (
          Shade_Face_len/2)*Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + (
          Shade_Face_len/2)*Modelica.Math.sin(Shade_Face_azi));

        d[i] = IRR.Irr.Components_Irr.Functions_Irr.dist(azmid[i], Shade_Face_azi,
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i,1], x0y0[i,2]),
          TgtFace_azi, IRR.Irr.Components_Irr.Functions_Irr.Di(
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i,1], x0y0[i,2]),
          Shade_Face_D, Shade_Face_theta, i, res, TgtFace_azi));

        H_sh[i] = IRR.Irr.Components_Irr.Functions_Irr.H_SH(
          IRR.Irr.Components_Irr.Functions_Irr.distDir(
          Sun_Az1[1],
          Shade_Face_azi,
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1] + Shade_Face_len*
          Modelica.Math.cos(Shade_Face_azi), x0y0[i, 2] + Shade_Face_len*
          Modelica.Math.sin(Shade_Face_azi)),
          TgtFace_azi, IRR.Irr.Components_Irr.Functions_Irr.Di(
          IRR.Irr.Components_Irr.Functions_Irr.tanInv(x0y0[i, 1], x0y0[i, 2]),
          Shade_Face_D, Shade_Face_theta, i, res, TgtFace_azi)),
          Shade_Face_H, Sun_Zen1[1]);

        for j in 1:k loop
          //SAngle[(i-1)*k+j] = Functions_Dif.SolidAngle(d[i], (j-1)*res/100, azmid[i], Shade_Face_azi, Shade_Face_len, Shade_Face_H);

          I[(i-1)*k+j] = IRR.Irr.Components_Irr.Functions_Irr.Frac(H_sh[i],
            winz+(j-1)*res/100);

          (SVF1[(i-1)*k+j], SVF3[(i-1)*k+j])=SVF1and3(x0y0[i,1], x0y0[i,2], Shade_Face_len, Shade_Face_azi,
          Shade_Face_H, winz+(j-1)*res/100, d[i], pii, SkyDifHor);

          SVF[(i-1)*k+j] = (SVF1[(i-1)*k+j]*SkyDifIso + I[(i-1)*k+j]*SkyDifCir +
            SVF3[(i-1)*k+j]*SkyDifHor)/HDifTil.HDifTil.HSkyDifTil;

        end for;

      end for;

      //fac1 = sum(SVF1)/(n*k);
      //fac3 = sum(SVF3)/n;
      //fac2 = sum(I)/(n*k);
      //SkyDifSH = fac2*SkyDifCir + fac1*SkyDifIso + fac3*SkyDifHor;

      connect(sim.weaDatBus, HDifTil.weaBus) annotation (Line(
          points={{-78.1,88},{-74,88},{-74,64},{-66,64}},
          color={255,204,51},
          thickness=0.5));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=31536000,
          Interval=600,
          __Dymola_fixedstepsize=600,
          __Dymola_Algorithm="Euler"));
    end ShObjWAlt;

    function SVF1and3
      "Calculates SVF1 (isotropic) and SVF3 (horizon) for skyDif shading"
      input Modelica.SIunits.Length x0, y0;
      input Modelica.SIunits.Length lenS;
      input Modelica.SIunits.Angle aziS;
      input Modelica.SIunits.Length hS;
      input Modelica.SIunits.Length zT;
      input Modelica.SIunits.Length d;
      input Modelica.SIunits.SolidAngle pii;
      input Real SDHor;
      output Real SVF1;
      output Real SVF3;

    protected
      Modelica.SIunits.Angle az0 = Irr.Components_Irr.Functions_Irr.tanInv(x0, y0);
      Modelica.SIunits.Angle az1 = Irr.Components_Irr.Functions_Irr.tanInv(x0+lenS*Modelica.Math.cos(aziS), y0+lenS*Modelica.Math.sin(aziS));
      Modelica.SIunits.Angle thetaH = if az0 > az1 then az0-az1 else az1-az0;
      Modelica.SIunits.Length H = hS-zT;
      Modelica.SIunits.Angle thetaV = if d>0 and H>0 then 2*Modelica.Math.atan(H/d) else 0;
      Modelica.SIunits.SolidAngle SA = 0.5*4*Modelica.Math.asin(Modelica.Math.sin(thetaH/2)*Modelica.Math.sin(thetaV/2));

    algorithm

      SVF3 :=if H>0 then (Modelica.Constants.pi-thetaH)/Modelica.Constants.pi else 1;
      SVF1 :=(pii-SA)/pii;

    end SVF1and3;
  end IrrAlt;
  annotation (uses(IDEAS(version="2.1.0"), Modelica(version="3.2.3")));
end IRR;

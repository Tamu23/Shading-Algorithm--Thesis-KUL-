within ;
package Tamu

  model SingleCell_V3 "Using Diode from PhotoVoltaics Library"

    parameter Modelica.SIunits.Current IRef=8.89 "Reference current at reference irradiance and reference temperature";
    parameter Modelica.SIunits.Irradiance irradianceRef=1000 "Reference solar irradiance";
    final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=+0.00053 "Temperature coefficient of reference current at TRref";
    parameter Modelica.SIunits.Resistance Rsh=20 "Resistance at temperature T_ref";
    parameter Modelica.SIunits.Resistance Rse=0.0055 "Resistance at temperature T_ref";
    parameter Real m=1 "Ideality factor of diode";
    parameter Modelica.SIunits.Resistance R=2.2 "Parallel ohmic resistance";
    parameter Modelica.SIunits.Temperature TRef=298.15 "Reference temperature";
    parameter Modelica.SIunits.Voltage VRef=0.6292 "Reference voltage > 0 at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaI=+0.00053 "Temperature coefficient of reference current at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaV=-0.00340 "Temperature coefficient of reference voltage at TRef*";

    PhotoVoltaics.Sources.Electrical.SignalCurrent i_source(
      useHeatPort=true,
      final irradianceRef=irradianceRef,
      final alphaRef=alphaRef,
      final IRef=IRef)         annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={50,0})));
    PhotoVoltaics.Components.Diodes.Diode Diode(
      useHeatPort=true,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      IRef=IRef,
      alphaI=alphaI,
      alphaV=alphaV) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,0})));
    Modelica.Electrical.Analog.Basic.Resistor Rshunt(R=Rsh) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-30,0})));
    Modelica.Electrical.Analog.Basic.Resistor Rseries(R=Rse) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-50,-10})));
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{120,-20},{80,20}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a
      annotation (Placement(transformation(extent={{-10,90},{10,110}})));

    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow
      prescribedHeatFlow
      annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
    Modelica.Blocks.Sources.RealExpression Power(y=0)
      annotation (Placement(transformation(extent={{-80,86},{-60,106}})));

    Modelica.Blocks.Sources.RealExpression Power1(y=(-1)*abs(pin_n.i)*abs(abs(
          pin_n.v) - abs(pin_p.v)))
      annotation (Placement(transformation(extent={{-80,66},{-60,86}})));
  equation
    connect(Diode.n, i_source.p)
      annotation (Line(points={{4.44089e-016,10},{4.44089e-016,10},{50,10}},
                                                         color={0,0,255}));
    connect(Diode.p, i_source.n)
      annotation (Line(points={{-6.66134e-016,-10},{-6.66134e-016,-10},{50,-10}},
                                                            color={0,0,255}));
    connect(Rshunt.n, Diode.n)
      annotation (Line(points={{-30,10},{-14,10},{4.44089e-016,10}},
                                                  color={0,0,255}));
    connect(Rshunt.p, Diode.p)
      annotation (Line(points={{-30,-10},{-14,-10},{-6.66134e-016,-10}},
                                                            color={0,0,255}));
    connect(Rseries.n, Rshunt.p)
      annotation (Line(points={{-40,-10},{-40,-10},{-30,-10}}, color={0,0,255}));
    connect(i_source.irradiance, u) annotation (Line(points={{57,-1.33227e-015},{72.5,
            -1.33227e-015},{72.5,0},{100,0}}, color={0,0,127}));
    connect(port_a, i_source.heatPort)
      annotation (Line(points={{0,100},{40,100},{40,0}},   color={191,0,0}));
    connect(Rseries.p, pin_p) annotation (Line(points={{-60,-10},{-80,-10},{-80,-60},
            {-100,-60}}, color={0,0,255}));
    connect(Rshunt.n, pin_n) annotation (Line(points={{-30,10},{-80,10},{-80,60},{
            -100,60}}, color={0,0,255}));
    connect(prescribedHeatFlow.port, port_a) annotation (Line(points={{-20,80},
            {0,80},{0,100},{0,100}}, color={191,0,0}));
    connect(port_a, Diode.heatPort)
      annotation (Line(points={{0,100},{10,100},{10,0}}, color={191,0,0}));
    connect(Power1.y, prescribedHeatFlow.Q_flow) annotation (Line(points={{-59,76},
            {-59,76},{-50,76},{-50,80},{-40,80}},
                                             color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-60,60},{60,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end SingleCell_V3;

  model String20cells
    parameter Modelica.SIunits.Current IRef=8.89     "Reference current at reference irradiance and reference temperature";
    parameter Modelica.SIunits.Irradiance irradianceRef=1000     "Reference solar irradiance";
    parameter Modelica.SIunits.Resistance Rsh=20 "Resistance at temperature T_ref";
    parameter Modelica.SIunits.Resistance Rse=0.0055     "Resistance at temperature T_ref";
    parameter Real Rcon = 10^(-3) "Resistance between cells due to interconnecting ribbon [Ohm]";
    parameter Real m=1 "Ideality factor of diode";
    parameter Modelica.SIunits.Resistance R=2.2 "Parallel ohmic resistance";
    parameter Modelica.SIunits.Temperature TRef=298.15 "Reference temperature";
    parameter Modelica.SIunits.Voltage VRef=0.6292     "Reference voltage > 0 at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaI=+0.00053     "Temperature coefficient of reference current at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaV=-0.00340     "Temperature coefficient of reference voltage at TRef*";
    final parameter Integer nCells=20 "Number of cells in one string";

    SingleCell_V3 C1(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,-190},{60,-170}})));

    SingleCell_V3 C2(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,-150},{60,-130}})));
    SingleCell_V3 C3(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,-110},{60,-90}})));
    SingleCell_V3 C4(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,-70},{60,-50}})));
    SingleCell_V3 C5(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
    SingleCell_V3 C6(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,10},{60,30}})));
    SingleCell_V3 C7(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,50},{60,70}})));
    SingleCell_V3 C8(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,90},{60,110}})));
    SingleCell_V3 C9(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,130},{60,150}})));
    SingleCell_V3 C10(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,170},{60,190}})));
    SingleCell_V3 C11(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,190},{-60,170}})));
    SingleCell_V3 C12(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,150},{-60,130}})));
    SingleCell_V3 C13(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,110},{-60,90}})));
    SingleCell_V3 C14(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,70},{-60,50}})));
    SingleCell_V3 C15(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,30},{-60,10}})));
    SingleCell_V3 C16(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,-10},{-60,-30}})));
    SingleCell_V3 C17(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,-50},{-60,-70}})));
    SingleCell_V3 C18(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,-90},{-60,-110}})));
    SingleCell_V3 C19(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,-130},{-60,-150}})));
    SingleCell_V3 C20(
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-40,-170},{-60,-190}})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon1(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,-160})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon2(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,-120})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon3(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,-80})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon4(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,-40})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon5(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,0})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon6(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,40})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon7(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,80})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon8(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,120})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon9(R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,160})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{90,-208},{110,-188}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
      annotation (Placement(transformation(extent={{-108,-210},{-88,-190}})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon10(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={2,186})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon11(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,160})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon12(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,120})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon13(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,80})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon14(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,40})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon15(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,0})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon16(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,-40})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon17(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,-80})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon18(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,-120})));
    Modelica.Electrical.Analog.Basic.Resistor Rcon19(
                                                    R=Rcon) annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-20,-160})));
    Modelica.Blocks.Interfaces.RealInput u[nCells]   annotation (Placement(transformation(
          extent={{20,-20},{-20,20}},
          rotation=90,
          origin={0,220})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a[nCells]
      annotation (Placement(transformation(extent={{-10,-210},{10,-190}})));
  equation
    connect(C1.pin_n, Rcon1.p) annotation (Line(points={{40,-174},{34,-174},{34,-170},
            {20,-170}}, color={0,0,255}));
    connect(Rcon1.n, C2.pin_p) annotation (Line(points={{20,-150},{34,-150},{34,-146},
            {40,-146}}, color={0,0,255}));
    connect(Rcon2.p, C2.pin_n) annotation (Line(points={{20,-130},{34,-130},{34,-134},
            {40,-134}}, color={0,0,255}));
    connect(Rcon2.n, C3.pin_p) annotation (Line(points={{20,-110},{34,-110},{34,-106},
            {40,-106}}, color={0,0,255}));
    connect(Rcon3.p, C3.pin_n) annotation (Line(points={{20,-90},{34,-90},{34,-94},
            {40,-94}},  color={0,0,255}));
    connect(Rcon3.n, C4.pin_p) annotation (Line(points={{20,-70},{34,-70},{34,-66},
            {40,-66}}, color={0,0,255}));
    connect(Rcon4.p, C4.pin_n) annotation (Line(points={{20,-50},{34,-50},{34,-54},
            {40,-54}}, color={0,0,255}));
    connect(Rcon4.n, C5.pin_p) annotation (Line(points={{20,-30},{34,-30},{34,-26},
            {40,-26}}, color={0,0,255}));
    connect(Rcon5.p, C5.pin_n) annotation (Line(points={{20,-10},{34,-10},{34,-14},
            {40,-14}}, color={0,0,255}));
    connect(Rcon5.n, C6.pin_p) annotation (Line(points={{20,10},{34,10},{34,14},{40,14}},
                                                             color={0,0,255}));
    connect(Rcon6.p, C6.pin_n) annotation (Line(points={{20,30},{34,30},{34,26},{40,
            26}}, color={0,0,255}));
    connect(Rcon6.n, C7.pin_p) annotation (Line(points={{20,50},{34,50},{34,54},{40,
            54}}, color={0,0,255}));
    connect(Rcon7.p, C7.pin_n) annotation (Line(points={{20,70},{34,70},{34,66},{40,
            66}}, color={0,0,255}));
    connect(Rcon7.n, C8.pin_p) annotation (Line(points={{20,90},{34,90},{34,94},{40,
            94}}, color={0,0,255}));
    connect(Rcon8.p, C8.pin_n) annotation (Line(points={{20,110},{34,110},{34,106},
            {40,106}},color={0,0,255}));
    connect(Rcon8.n, C9.pin_p) annotation (Line(points={{20,130},{34,130},{34,134},
            {40,134}}, color={0,0,255}));
    connect(Rcon9.n, C10.pin_p) annotation (Line(points={{20,170},{34,170},{34,174},
            {40,174}}, color={0,0,255}));
    connect(Rcon9.p, C9.pin_n) annotation (Line(points={{20,150},{36,150},{36,146},
            {40,146}}, color={0,0,255}));
    connect(C1.pin_p, pin_p) annotation (Line(points={{40,-186},{40,-186},{40,-200},
            {100,-200},{100,-198}}, color={0,0,255}));
    connect(C20.pin_n, pin_n) annotation (Line(points={{-40,-186},{-40,-186},{-40,
            -200},{-98,-200}}, color={0,0,255}));
    connect(Rcon10.p, C10.pin_n) annotation (Line(points={{12,186},{28,186},{40,186}}, color={0,0,255}));
    connect(Rcon10.n, C11.pin_p) annotation (Line(points={{-8,186},{-24,186},{-40,186}}, color={0,0,255}));
    connect(C11.pin_n, Rcon11.p) annotation (Line(points={{-40,174},{-32,174},{-32,
            170},{-20,170}}, color={0,0,255}));
    connect(C12.pin_p, Rcon11.n) annotation (Line(points={{-40,146},{-30,146},{-30,
            150},{-20,150}}, color={0,0,255}));
    connect(C12.pin_n, Rcon12.p) annotation (Line(points={{-40,134},{-30,134},{-30,
            130},{-20,130}}, color={0,0,255}));
    connect(C13.pin_p, Rcon12.n) annotation (Line(points={{-40,106},{-30,106},{-30,
            110},{-20,110}}, color={0,0,255}));
    connect(C13.pin_n, Rcon13.p) annotation (Line(points={{-40,94},{-30,94},{-30,90},
            {-20,90}}, color={0,0,255}));
    connect(C14.pin_p, Rcon13.n) annotation (Line(points={{-40,66},{-30,66},{-30,70},
            {-20,70}}, color={0,0,255}));
    connect(C14.pin_n, Rcon14.p) annotation (Line(points={{-40,54},{-30,54},{-30,50},
            {-20,50}}, color={0,0,255}));
    connect(C15.pin_p, Rcon14.n) annotation (Line(points={{-40,26},{-30,26},{-30,30},
            {-20,30}}, color={0,0,255}));
    connect(C15.pin_n, Rcon15.p) annotation (Line(points={{-40,14},{-30,14},{-30,10},
            {-20,10}}, color={0,0,255}));
    connect(C16.pin_p, Rcon15.n) annotation (Line(points={{-40,-14},{-30,-14},{-30,
            -10},{-20,-10}}, color={0,0,255}));
    connect(C16.pin_n, Rcon16.p) annotation (Line(points={{-40,-26},{-30,-26},{-30,
            -30},{-20,-30}}, color={0,0,255}));
    connect(C17.pin_p, Rcon16.n) annotation (Line(points={{-40,-54},{-30,-54},{-30,
            -50},{-20,-50}}, color={0,0,255}));
    connect(C17.pin_n, Rcon17.p) annotation (Line(points={{-40,-66},{-32,-66},{-32,
            -70},{-20,-70}}, color={0,0,255}));
    connect(C18.pin_p, Rcon17.n) annotation (Line(points={{-40,-94},{-30,-94},{-30,
            -90},{-20,-90}}, color={0,0,255}));
    connect(C18.pin_n, Rcon18.p) annotation (Line(points={{-40,-106},{-30,-106},{-30,
            -110},{-20,-110}}, color={0,0,255}));
    connect(C19.pin_p, Rcon18.n) annotation (Line(points={{-40,-134},{-30,-134},{-30,
            -130},{-20,-130}}, color={0,0,255}));
    connect(C19.pin_n, Rcon19.p) annotation (Line(points={{-40,-146},{-32,-146},{-32,
            -150},{-20,-150}}, color={0,0,255}));
    connect(C20.pin_p, Rcon19.n) annotation (Line(points={{-40,-174},{-30,-174},{-30,
            -170},{-20,-170}}, color={0,0,255}));

    connect(u[1], C1.u);
    connect(u[2], C2.u);
    connect(u[3], C3.u);
    connect(u[4], C4.u);
    connect(u[5], C5.u);
    connect(u[6], C6.u);
    connect(u[7], C7.u);
    connect(u[8], C8.u);
    connect(u[9], C9.u);
    connect(u[10], C10.u);
    connect(u[11], C11.u);
    connect(u[12], C12.u);
    connect(u[13], C13.u);
    connect(u[14], C14.u);
    connect(u[15], C15.u);
    connect(u[16], C16.u);
    connect(u[17], C17.u);
    connect(u[18], C18.u);
    connect(u[19], C19.u);
    connect(u[20], C20.u);

    connect(port_a[1], C1.port_a);
    connect(port_a[2], C2.port_a);
    connect(port_a[3], C3.port_a);
    connect(port_a[4], C4.port_a);
    connect(port_a[5], C5.port_a);
    connect(port_a[6], C6.port_a);
    connect(port_a[7], C7.port_a);
    connect(port_a[8], C8.port_a);
    connect(port_a[9], C9.port_a);
    connect(port_a[10], C10.port_a);
    connect(port_a[11], C11.port_a);
    connect(port_a[12], C12.port_a);
    connect(port_a[13], C13.port_a);
    connect(port_a[14], C14.port_a);
    connect(port_a[15], C15.port_a);
    connect(port_a[16], C16.port_a);
    connect(port_a[17], C17.port_a);
    connect(port_a[18], C18.port_a);
    connect(port_a[19], C19.port_a);
    connect(port_a[20], C20.port_a);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-200},
              {100,200}}), graphics={
          Rectangle(
            extent={{-46,-144},{-18,-174}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,-108},{-18,-138}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,-36},{-18,-66}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,-72},{-18,-102}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,-144},{48,-174}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,-108},{48,-138}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,-36},{48,-66}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,-72},{48,-102}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,0},{-18,-30}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,36},{-18,6}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,108},{-18,78}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,72},{-18,42}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,0},{48,-30}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,36},{48,6}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,108},{48,78}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,72},{48,42}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,180},{-18,150}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-46,144},{-18,114}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,180},{48,150}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,144},{48,114}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-200},{100,200}})));
  end String20cells;

  model PV60cellModule

    parameter Real Rcon = 10^(-3) "Resistance between cells due to interconnecting ribbon [Ohm]";
    parameter Modelica.SIunits.Current IRef=8.89     "Reference current at reference irradiance and reference temperature";
    parameter Modelica.SIunits.Irradiance irradianceRef=1000     "Reference solar irradiance";
    final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=alphaI     "Temperature coefficient of reference current at TRref";
    parameter Modelica.SIunits.Resistance Rsh=20 "Resistance at temperature T_ref";
    parameter Modelica.SIunits.Resistance Rse=0.0055 "Resistance at temperature T_ref";
    parameter Real m=1 "Ideality factor of diode";
    parameter Modelica.SIunits.Resistance R=2.2 "Parallel ohmic resistance";
    parameter Modelica.SIunits.Temperature TRef=298.15 "Reference temperature";
    parameter Modelica.SIunits.Voltage VRef=0.6292     "Reference voltage > 0 at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaI=+0.00053     "Temperature coefficient of reference current at TRef";
    parameter Modelica.SIunits.LinearTemperatureCoefficient alphaV=-0.00340     "Temperature coefficient of reference voltage at TRef*";
    parameter Real Rstr = 6.4*10^(-3) "Resistance between strings due to interconnecting ribbon [Ohm]";
    final parameter Integer nCells=60;

    String20cells string20cells1(
      Rcon=Rcon,
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-60,10},{-40,50}})));

    String20cells string20cells2(
      Rcon=Rcon,
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{-10,10},{10,50}})));
    String20cells string20cells3(
      Rcon=Rcon,
      IRef=IRef,
      irradianceRef=irradianceRef,
      Rsh=Rsh,
      Rse=Rse,
      m=m,
      R=R,
      TRef=TRef,
      VRef=VRef,
      alphaI=alphaI,
      alphaV=alphaV)
      annotation (Placement(transformation(extent={{40,10},{60,50}})));
    Modelica.Blocks.Interfaces.RealInput u1[nCells] annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,110})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a1[nCells]
      annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
    Modelica.Electrical.Analog.Basic.Resistor Rstring1(R=Rstr)
      annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
    Modelica.Electrical.Analog.Basic.Resistor Rstring(R=Rstr) annotation (Placement(transformation(extent={{34,-10},
              {14,10}})));
    Modelica.Electrical.Analog.Basic.Resistor Rstring2(
                                                      R=Rstr) annotation (Placement(transformation(extent={{-14,-10},
              {-34,10}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin n1                 "Negative pin"     annotation (Placement(transformation(extent={{-110,50},
              {-90,70}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
  equation

    connect(Rstring1.n, string20cells3.pin_p)     annotation (Line(points={{0,-60},{60,-60},{60,10.2}},
                                                          color={0,0,255}));
    connect(string20cells2.pin_p, Rstring.n)     annotation (Line(points={{10,10.2},{14,10.2},{14,0}},
                                                        color={0,0,255}));
    connect(string20cells3.pin_n, Rstring.p)     annotation (Line(points={{40.2,10},{36,10},{36,0},{34,0}},
                                                               color={0,0,255}));
    connect(string20cells1.pin_p, Rstring2.n) annotation (Line(points={{-40,10.2},
            {-38,10.2},{-38,0},{-34,0}},
                                  color={0,0,255}));
    connect(string20cells2.pin_n, Rstring2.p) annotation (Line(points={{-9.8,10},{
            -12,10},{-12,0},{-14,0}},
                                  color={0,0,255}));
    connect(string20cells1.pin_n, n1) annotation (Line(points={{-59.8,10},{-70,10},
            {-70,60},{-100,60}},color={0,0,255}));
    connect(Rstring1.p, pin_p) annotation (Line(points={{-20,-60},{-100,-60}},
                   color={0,0,255}));

            for i in 1:20 loop
      connect(u1[i], string20cells1.u[i]);
      connect(port_a1[i], string20cells1.port_a[i]);

      connect(u1[i+20], string20cells2.u[i]);
      connect(port_a1[i+20], string20cells2.port_a[i]);

      connect(u1[i+40], string20cells3.u[i]);
      connect(port_a1[i+40], string20cells3.port_a[i]);

            end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-58,-82},{-40,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,-62},{-40,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,-42},{-40,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,-22},{-40,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,-2},{-40,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,18},{-40,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,38},{-40,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,58},{-40,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,78},{-40,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-58,98},{-40,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-82},{-20,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-62},{-20,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-42},{-20,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-22},{-20,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-2},{-20,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,18},{-20,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,38},{-20,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,58},{-20,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,78},{-20,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,98},{-20,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,-82},{0,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,-62},{0,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,-42},{0,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,-22},{0,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,-2},{0,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,18},{0,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,38},{0,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,58},{0,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,78},{0,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,98},{0,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,-82},{20,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,-62},{20,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,-42},{20,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,-22},{20,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,-2},{20,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,18},{20,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,38},{20,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,58},{20,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,78},{20,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{2,98},{20,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,-82},{40,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,-62},{40,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,-42},{40,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,-22},{40,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,-2},{40,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,18},{40,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,38},{40,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,58},{40,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,78},{40,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{22,98},{40,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,-82},{60,-100}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,-62},{60,-80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,-42},{60,-60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,-22},{60,-40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,-2},{60,-20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,18},{60,0}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,38},{60,20}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,58},{60,40}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,78},{60,60}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{42,98},{60,80}},
            lineColor={28,108,200},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end PV60cellModule;

  model TemperatureModel

    parameter Real k;
    parameter Integer nCells=60;

    Modelica.Blocks.Interfaces.RealInput I[nCells]
      annotation (Placement(transformation(extent={{-120,20},{-80,60}})));
    Modelica.Blocks.Interfaces.RealOutput Tpv[nCells]
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Modelica.Blocks.Interfaces.RealInput Tamb
      annotation (Placement(transformation(extent={{-120,-60},{-80,-20}})));

  equation

     for i in 1:nCells loop
     Tpv[i] = Tamb + k*I[i];
     end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TemperatureModel;

  model Example1

    parameter Integer nCells=60;

      Modelica.Blocks.Sources.RealExpression Power(y=sink.v*sink.i)
      annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
  protected
    ShadingComponents.ShIrr1 shIrr(TgtFace_azi=0.75049157835756)
      annotation (Placement(transformation(extent={{-66,50},{-46,70}})));
    inner IDEAS.BoundaryConditions.SimInfoManager sim
      annotation (Placement(transformation(extent={{78,78},{98,98}})));
      TemperatureModel temperatureModel(k=0.0054)
      annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
    PV60cellModule pV60cellModule
      annotation (Placement(transformation(extent={{40,-70},{60,-50}})));
    PhotoVoltaics.Components.Blocks.MPTrackerSample mpTracker(
      n=100,
      ImpRef=8,
      VmpRef=12,
      samplePeriod=10)
      annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage sink annotation (Placement(
          transformation(
          origin={0,-60},
          extent={{10,10},{-10,-10}},
          rotation=270)));

    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature[nCells]
      annotation (Placement(transformation(extent={{20,-20},{40,0}})));
    Modelica.Blocks.Sources.RealExpression Tamb(y=sim.weaBus.Te)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Modelica.Electrical.Spice3.Basic.Ground ground     annotation (Placement(transformation(extent={{-30,
              -100},{-10,-80}})));

  equation
    connect(sink.p, pV60cellModule.pin_p) annotation (Line(points={{-1.77636e-015,
            -70},{0,-70},{0,-80},{40,-80},{40,-66}}, color={0,0,255}));
    connect(sink.n, pV60cellModule.n1) annotation (Line(points={{1.77636e-015,-50},
            {1.77636e-015,-40},{40,-40},{40,-54}}, color={0,0,255}));
    connect(mpTracker.vRef, sink.v)
      annotation (Line(points={{-19,-60},{-12,-60}},color={0,0,127}));
    connect(Power.y, mpTracker.power)
      annotation (Line(points={{-59,-60},{-42,-60}}, color={0,0,127}));
    connect(Tamb.y, temperatureModel.Tamb) annotation (Line(points={{-59,0},{-50,0},
            {-50,-14},{-20,-14}}, color={0,0,127}));
    connect(temperatureModel.Tpv, prescribedTemperature.T)
      annotation (Line(points={{1,-10},{1,-10},{18,-10}}, color={0,0,127}));
    connect(prescribedTemperature.port, pV60cellModule.port_a1) annotation (Line(
          points={{40,-10},{40,-10},{80,-10},{80,-80},{50,-80},{50,-70}}, color={191,
            0,0}));
    connect(sink.n, ground.p) annotation (Line(points={{0,-50},{-10,-50},{-10,-80},
            {-20,-80}}, color={0,0,255}));
    connect(shIrr.y, temperatureModel.I)
      annotation (Line(points={{-45,60},{-45,-6},{-20,-6}}, color={0,0,127}));
    connect(shIrr.y, pV60cellModule.u1)
      annotation (Line(points={{-45,60},{50,60},{50,-49}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{-162,92},{-96,62}},
            lineColor={28,108,200},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="You will have to substitute 
these two components 
by connections to the infosim"),
          Line(points={{-96,78},{-82,70}}, color={28,108,200}),
          Line(points={{-96,78},{-84,2}}, color={28,108,200}),
          Text(
            extent={{86,-14},{152,-44}},
            lineColor={28,108,200},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="This is the PV module.
It has 60 cells. So, you need
to input 60 values of irradiance,
one for each cell"),
          Line(points={{90,-38},{58,-56}}, color={28,108,200}),
          Rectangle(
            extent={{-86,-34},{16,-98}},
            lineColor={255,128,0},
            lineThickness=1),
          Text(
            extent={{-84,-74},{-30,-96}},
            lineColor={255,128,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="MPP tracking.
You will always need 
these components together
with the PV module")}),
      experiment(
        StopTime=31536000,
        Interval=600,
        __Dymola_fixedstepsize=30,
        __Dymola_Algorithm="Dassl"),
      __Dymola_experimentSetupOutput(events=false));
  end Example1;

  package ShadingComponents
    model ShIrr1 "Calculates shaded irradiation (modular shading)"

      parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
      constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
      parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Boolean shDir = false "Set true to multiply with external shading factor";
      parameter Boolean shDif = false "Set true to multiply with external shading factor";
      final parameter Integer nCells = 60;

      Modelica.Blocks.Interfaces.RealOutput y[nCells]
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      IDEAS.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil(
        til=TgtFace_tilt,
        lat=lat,
        azi=TgtFace_azi,
        outSkyCon=true,
        outGroCon=true)
        annotation (Placement(transformation(extent={{-60,38},{-40,58}})));
      DirFac dirFac(shDir=shDir)
                    annotation (Placement(transformation(extent={{-8,64},{12,84}})));
      SkyDifFac skyDifFac(shDif=shDif)
        annotation (Placement(transformation(extent={{-8,32},{12,52}})));
      inner IDEAS.BoundaryConditions.SimInfoManager sim
        annotation (Placement(transformation(extent={{-100,78},{-80,98}})));
      IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
        til=TgtFace_tilt,
        lat=lat,
        azi=TgtFace_azi)
        annotation (Placement(transformation(extent={{-50,70},{-30,90}})));
    equation

       for i in 1:nCells loop
         y[i] = dirFac.y + skyDifFac.y + HDifTil.HGroDifTil;
       end for;

      connect(HDifTil.HSkyDifTil, skyDifFac.u) annotation (Line(points={{-39,54},{-26,
              54},{-26,42},{-8.6,42}}, color={0,0,127}));
      connect(sim.weaDatBus, HDifTil.weaBus) annotation (Line(
          points={{-80.1,88},{-70,88},{-70,48},{-60,48}},
          color={255,204,51},
          thickness=0.5));
      connect(sim.weaDatBus, HDirTil.weaBus) annotation (Line(
          points={{-80.1,88},{-66,88},{-66,80},{-50,80}},
          color={255,204,51},
          thickness=0.5));
      connect(dirFac.u, HDirTil.H) annotation (Line(points={{-8.6,74},{-18,74},
              {-18,80},{-29,80}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=31536000));
    end ShIrr1;

    model DirFac "Multiplication factor for external shading - Dir"
      parameter Boolean shDir "Set true to multiply with external shading factor";
      Real fac;

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      //Import appropriate shading factor below:
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/3/3b/3bforPV1.mat",
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
      parameter Boolean shDif "Set true to multiply with external shading factor";
      Real fac;

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      //Import appropriate shading factor below:
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/3/3b/3bforPV1.mat",
        columns={2})
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
    equation
      fac = if shDif then combiTimeTable.y[1] else 1;
      y=u*fac;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end SkyDifFac;

    model ShIrr60 "Calculates shaded irradiation (partial shading)"

      parameter Modelica.SIunits.Angle lat=Modelica.SIunits.Conversions.from_deg(51) "Site latitude";
      constant Modelica.SIunits.Angle TgtFace_tilt=Modelica.SIunits.Conversions.from_deg(90) "Tilt angle, in deg [90 for walls]" annotation(HideResult=true);
      parameter Modelica.SIunits.Angle TgtFace_azi(min=-180,max=180)=Modelica.SIunits.Conversions.from_deg(43) "Azimuth angle";
      /* Facade origin is to the left (CCW) of azimuth direction, end point to the right (CW) */
      parameter Boolean shDir = false "Set true to multiply with external shading factor";
      parameter Boolean shDif = false "Set true to multiply with external shading factor";

      Modelica.Blocks.Interfaces.RealOutput y[60]
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      inner IDEAS.BoundaryConditions.SimInfoManager sim
        annotation (Placement(transformation(extent={{-98,78},{-78,98}})));
      IDEAS.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil(
        til=TgtFace_tilt,
        lat=lat,
        azi=TgtFace_azi,
        outSkyCon=true,
        outGroCon=true)
        annotation (Placement(transformation(extent={{-60,38},{-40,58}})));
      DirFac60 dirFac60(shDir=shDir)
        annotation (Placement(transformation(extent={{-10,68},{10,88}})));
      SkyDifFac60 skyDifFac60(shDif=shDif)
        annotation (Placement(transformation(extent={{-8,32},{12,52}})));
      IDEAS.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
        til=TgtFace_tilt,
        lat=lat,
        azi=TgtFace_azi)
        annotation (Placement(transformation(extent={{-50,72},{-30,92}})));
    equation

       for i in 1:60 loop
         y[i] = dirFac60.y[i] + skyDifFac60.y[i] + HDifTil.HGroDifTil;
       end for;

      connect(sim.weaDatBus, HDifTil.weaBus) annotation (Line(
          points={{-78.1,88},{-70,88},{-70,48},{-60,48}},
          color={255,204,51},
          thickness=0.5));
      connect(skyDifFac60.u, HDifTil.HSkyDifTil) annotation (Line(points={{-8.6,42},
              {-24,42},{-24,54},{-39,54}}, color={0,0,127}));
      connect(sim.weaDatBus, HDirTil.weaBus) annotation (Line(
          points={{-78.1,88},{-64,88},{-64,82},{-50,82}},
          color={255,204,51},
          thickness=0.5));
      connect(dirFac60.u, HDirTil.H) annotation (Line(points={{-10.6,78},{-20,
              78},{-20,82},{-29,82}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=31536000));
    end ShIrr60;

    model DirFac60 "Multiplication factor for external shading - Dir"
      parameter Boolean shDir "Set true to multiply with external shading factor";
      Real fac[60];

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y[60]
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      //Import appropriate shading factor below:
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/4/4b/4bforPV60C1D10.mat",
        columns=62:121)
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
    equation

      for i in 1:60 loop
        fac[i] = if shDir then combiTimeTable.y[i] else 1;
        y[i]=u*fac[i];
      end for;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=31536000, Interval=600),
        __Dymola_experimentSetupOutput(events=false));
    end DirFac60;

    model SkyDifFac60
      "Multiplication factor for external shading - Sky Diff"
      parameter Boolean shDif "Set true to multiply with external shading factor";
      Real fac[60];

      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-126,-20},{-86,20}})));
      Modelica.Blocks.Interfaces.RealOutput y[60]
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      //Import appropriate shading factor below:
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,
        tableName="data",
        fileName="G:/Modelica Results/Post-Process New/4/4b/4bforPV60C1D10.mat",
        columns=2:61)
        annotation (Placement(transformation(extent={{-8,50},{12,70}})));
    equation

      for i in 1:60 loop
        fac[i] = if shDif then combiTimeTable.y[i] else 1;
        y[i]=u*fac[i];
      end for;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end SkyDifFac60;

  end ShadingComponents;

  model Example60

    parameter Integer nCells=60;

    Modelica.Blocks.Sources.RealExpression Power(y=sink.v*sink.i)
      annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
  protected
    ShadingComponents.ShIrr60 shIrr60_1(
      TgtFace_azi=0.75049157835756,
      shDir=true,
      shDif=true)
      annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
    PV60cellModule pV60cellModule
      annotation (Placement(transformation(extent={{40,-70},{60,-50}})));
    PhotoVoltaics.Components.Blocks.MPTrackerSample mpTracker(
      n=100,
      ImpRef=8,
      VmpRef=12,
      samplePeriod=10)
      annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage sink annotation (Placement(
          transformation(
          origin={0,-60},
          extent={{10,10},{-10,-10}},
          rotation=270)));
    TemperatureModel temperatureModel(k=0.0054)
      annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature[nCells]
      annotation (Placement(transformation(extent={{20,-20},{40,0}})));
    Modelica.Blocks.Sources.RealExpression Tamb(y=sim.weaBus.Te)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Modelica.Electrical.Spice3.Basic.Ground ground     annotation (Placement(transformation(extent={{-30,
              -100},{-10,-80}})));
    inner IDEAS.BoundaryConditions.SimInfoManager sim
      annotation (Placement(transformation(extent={{74,76},{94,96}})));

  equation
    connect(sink.p, pV60cellModule.pin_p) annotation (Line(points={{-1.77636e-015,
            -70},{0,-70},{0,-80},{40,-80},{40,-66}}, color={0,0,255}));
    connect(sink.n, pV60cellModule.n1) annotation (Line(points={{1.77636e-015,-50},
            {1.77636e-015,-40},{40,-40},{40,-54}}, color={0,0,255}));
    connect(mpTracker.vRef, sink.v)
      annotation (Line(points={{-19,-60},{-12,-60}},color={0,0,127}));
    connect(Power.y, mpTracker.power)
      annotation (Line(points={{-59,-60},{-42,-60}}, color={0,0,127}));
    connect(Tamb.y, temperatureModel.Tamb) annotation (Line(points={{-59,0},{-50,0},
            {-50,-14},{-20,-14}}, color={0,0,127}));
    connect(temperatureModel.Tpv, prescribedTemperature.T)
      annotation (Line(points={{1,-10},{1,-10},{18,-10}}, color={0,0,127}));
    connect(prescribedTemperature.port, pV60cellModule.port_a1) annotation (Line(
          points={{40,-10},{40,-10},{80,-10},{80,-80},{50,-80},{50,-70}}, color={191,
            0,0}));
    connect(sink.n, ground.p) annotation (Line(points={{0,-50},{-10,-50},{-10,-80},
            {-20,-80}}, color={0,0,255}));
    connect(temperatureModel.I, shIrr60_1.y) annotation (Line(points={{-20,-6},{-34,
            -6},{-34,60},{-47,60}}, color={0,0,127}));
    connect(shIrr60_1.y, pV60cellModule.u1)
      annotation (Line(points={{-47,60},{50,60},{50,-49}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{-162,92},{-96,62}},
            lineColor={28,108,200},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="You will have to substitute 
these two components 
by connections to the infosim"),
          Line(points={{-96,78},{-82,70}}, color={28,108,200}),
          Line(points={{-96,78},{-84,2}}, color={28,108,200}),
          Text(
            extent={{86,-14},{152,-44}},
            lineColor={28,108,200},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="This is the PV module.
It has 60 cells. So, you need
to input 60 values of irradiance,
one for each cell"),
          Line(points={{90,-38},{58,-56}}, color={28,108,200}),
          Rectangle(
            extent={{-86,-34},{16,-98}},
            lineColor={255,128,0},
            lineThickness=1),
          Text(
            extent={{-84,-74},{-30,-96}},
            lineColor={255,128,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            textString="MPP tracking.
You will always need 
these components together
with the PV module")}),
      experiment(StopTime=31536000, Interval=600),
      __Dymola_experimentSetupOutput(events=false));
  end Example60;

  annotation (uses(IDEAS(version="1.0.0"),
      PhotoVoltaics(version="X.X.X"),
      Modelica(version="3.2.3")));
end Tamu;

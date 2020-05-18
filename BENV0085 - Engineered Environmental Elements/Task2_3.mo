within ;
package Task2_3
  package Task2a
    "Collection of models that illustrate model use and test models"
    extends Modelica.Icons.ExamplesPackage;

    model ElectricEIRConst "Chiller Electric EIR with Water - Air Cooling Coil"
      extends Modelica.Icons.Example;
      extends Task2_3.Task2c.BaseClasses.PartialElectric(
        P_nominal=-per.QEva_flow_nominal/per.COP_nominal,
        mEva_flow_nominal=per.mEva_flow_nominal,
        mCon_flow_nominal=per.mCon_flow_nominal,
        sou1(
          redeclare package Medium = Medium1,
          m_flow=1.2*76.1,
          nPorts=1),
        TSetChi(k=TsetChi),
        sin1(p=101325));
      replaceable package MediumA = Buildings.Media.Air "Medium model";
      replaceable package MediumW = Buildings.Media.Water "Medium model";
      parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal=16 "Nominal mass flow rate at fan";
      parameter Modelica.SIunits.MassFlowRate mCHW_flow_nominal=32 "Nominal mass flow rate at chilled water";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal = mCHW_flow_nominal
        "Nominal mass flow rate";
      parameter Modelica.SIunits.Temperature TsetChi=280.15 "Chilled water set point";
      parameter Task2_3.ChillerData.ElectricEIRChiller_30XA220 per
        "Chiller performance data"
        annotation (Placement(transformation(extent={{70,76},{94,100}})));

      Task2_3.Task2c.ElectricEIRCW Chiller(
        redeclare package Medium2 = Medium2,
        per=per,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp1_nominal=6000,
        dp2_nominal=6000,
        redeclare package Medium1 = Medium1) "Chiller model"
        annotation (Placement(transformation(extent={{-2,36},{18,56}})));

      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaData(filNam=
            "C:/Users/BT/OneDrive - University College London/Engineered Environmental Elements/3E COURSEWORK/modelica-buildings-master/Task/ESP_Barcelona.081810_SWEC.mos")
        annotation (Placement(transformation(extent={{138,-62},{118,-42}})));
      Modelica.Blocks.Sources.Constant mPumpFlo1(k=mCHW_flow_nominal)
        "Mass flow rate of pump"
        annotation (Placement(transformation(extent={{94,-16},{80,-2}})));
      Buildings.Fluid.Sources.FixedBoundary sinA(
        nPorts=1,
        redeclare package Medium = MediumA,
        use_T=true)
        annotation (Placement(transformation(extent={{-92,-62},{-72,-42}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TAirOut(
                               redeclare package Medium = MediumA,
          m_flow_nominal=mAir_flow_nominal)
        "Temperature of air leaving cooling coil" annotation (Placement(
            transformation(
            extent={{10,10},{-10,-10}},
            rotation=0,
            origin={-40,-52})));
      Buildings.Fluid.Sources.Outside Weather_Data(nPorts=1, redeclare package
          Medium = MediumA)
        annotation (Placement(transformation(extent={{94,-62},{74,-42}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow Fan(
        redeclare package Medium = MediumA,
        m_flow_nominal=mAir_flow_nominal,
        nominalValuesDefineDefaultPressureCurve=true,
        dp_nominal=600)       annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={50,-52})));
      Buildings.Fluid.Movers.FlowControlled_m_flow ChillerPump(
        redeclare package Medium = Medium2,
        nominalValuesDefineDefaultPressureCurve=true,
        m_flow_nominal=mCHW_flow_nominal,
        dp_nominal=179352)     annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={38,14})));
      Modelica.Blocks.Sources.Constant mFanFlo(k=mAir_flow_nominal)
        "Mass flow rate of fan" annotation (Placement(transformation(extent={{12,-90},
                {26,-76}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_OUT(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water leaving chiller " annotation (Placement(
            transformation(
            extent={{-7,7},{7,-7}},
            rotation=270,
            origin={-29,15})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_IN(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water returning to chiller " annotation (Placement(
            transformation(
            extent={{-8,8},{8,-8}},
            rotation=90,
            origin={38,-18})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TAirFan(redeclare package
          Medium = MediumA, m_flow_nominal=mAir_flow_nominal)
        "Temperature of air leaving fan" annotation (Placement(transformation(
            extent={{5,6},{-5,-6}},
            rotation=0,
            origin={27,-52})));
      Modelica.Blocks.Continuous.Integrator integratorChi
        annotation (Placement(transformation(extent={{36,78},{50,92}})));
      Modelica.Blocks.Continuous.Integrator integratorPump
        annotation (Placement(transformation(extent={{80,14},{94,28}})));
      Modelica.Blocks.Continuous.Integrator integratorFan
        annotation (Placement(transformation(extent={{-4,-76},{-18,-62}})));
      Buildings.Fluid.HeatExchangers.WetCoilCounterFlow CoolingCoil(
        redeclare package Medium1 = Medium2,
        redeclare package Medium2 = MediumA,
        dp2_nominal=249*3,
        dp1_nominal=1000 + 89580,
        m2_flow_nominal=16,
        UA_nominal=mAir_flow_nominal*1006*5,
        m1_flow_nominal=32)
        annotation (Placement(transformation(extent={{-8,-56},{12,-36}})));
      Buildings.Fluid.Storage.ExpansionVessel expVesChi(V_start=1, redeclare
          package Medium = Medium2)
        annotation (Placement(transformation(extent={{-22,-24},{-6,-9}})));
    equation
      connect(Chiller.port_b1, res1.port_a) annotation (Line(
          points={{18,52},{34,52}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(Chiller.on, greaterThreshold.y) annotation (Line(
          points={{-4,49},{-10,49},{-10,86},{-25,86}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(TSetChi.y, Chiller.TSet) annotation (Line(points={{-75,2},{-50,2},
              {-50,43},{-4,43}}, color={0,0,127}));
      connect(TAirOut.port_b,sinA. ports[1])
        annotation (Line(points={{-50,-52},{-72,-52}}, color={0,127,255}));
      connect(Weather_Data.ports[1], Fan.port_a)
        annotation (Line(points={{74,-52},{60,-52}}, color={0,127,255}));
      connect(mFanFlo.y, Fan.m_flow_in)
        annotation (Line(points={{26.7,-83},{50,-83},{50,-64}}, color={0,0,127}));
      connect(ChillerPump.m_flow_in, mPumpFlo1.y) annotation (Line(points={{50,14},{
              66,14},{66,-9},{79.3,-9}}, color={0,0,127}));

      connect(Weather_Data.weaBus, weaData.weaBus) annotation (Line(
          points={{94,-51.8},{100,-51.8},{100,-52},{118,-52}},
          color={255,204,51},
          thickness=0.5));

      connect(Fan.port_b, TAirFan.port_a) annotation (Line(points={{40,-52},{32,-52}},
                                       color={0,127,255}));
      connect(Chiller.P, integratorChi.u) annotation (Line(points={{19,55},{20,
              55},{20,85},{34.6,85}}, color={0,0,127}));
      connect(Fan.P, integratorFan.u) annotation (Line(points={{39,-61},{44,-61},{44,
              -69},{-2.6,-69}},       color={0,0,127}));
      connect(CoolingCoil.port_b2, TAirOut.port_a)
        annotation (Line(points={{-8,-52},{-30,-52}}, color={0,127,255}));
      connect(TAirFan.port_b, CoolingCoil.port_a2)
        annotation (Line(points={{22,-52},{12,-52}}, color={0,127,255}));
      connect(TCHW_IN.port_a, CoolingCoil.port_b1) annotation (Line(points={{38,
              -26},{38,-40},{12,-40}}, color={0,127,255}));
      connect(CoolingCoil.port_a1, TCHW_OUT.port_b) annotation (Line(points={{-8,
              -40},{-29,-40},{-29,8}}, color={0,127,255}));
      connect(expVesChi.port_a, TCHW_OUT.port_b) annotation (Line(points={{-14,
              -24},{-14,-34},{-29,-34},{-29,8}}, color={0,127,255}));
      connect(Chiller.port_b2, TCHW_OUT.port_a) annotation (Line(points={{-2,40},
              {-29,40},{-29,22}}, color={0,127,255}));
      connect(ChillerPump.P, integratorPump.u)
        annotation (Line(points={{47,25},{47,21},{78.6,21}},color={0,0,127}));
      connect(TCHW_IN.port_b, ChillerPump.port_a)
        annotation (Line(points={{38,-10},{38,4}}, color={0,127,255}));
      connect(Chiller.port_a2, ChillerPump.port_b)
        annotation (Line(points={{18,40},{38,40},{38,24}}, color={0,127,255}));
      connect(sou1.weaBus, weaData.weaBus) annotation (Line(
          points={{-90,52.2},{-102,52.2},{-102,-98},{104,-98},{104,-52},{118,-52}},
          color={255,204,51},
          thickness=0.5));
      connect(sou1.ports[1], Chiller.port_a1)
        annotation (Line(points={{-70,52},{-2,52}}, color={0,127,255}));
      annotation (
    experiment(
          StartTime=20044800,
          StopTime=20649600,
          Interval=3600,
          Tolerance=1e-06),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Chillers/Examples/ElectricEIR.mos"
            "Simulate and plot"),
        Documentation(info="<html>
    
    <pre>
    <pre>                 
NOTE: when changing chiller model the following parameters should be updated:
1. mCHW_flow_nominal should be changed in the text layer of the main model (e.g ElectricEIRConst), this should be the same value as 
'mEva_flow_nominal'.

2. In the diagram model, double click on 'coocoi', update 'm1_flow_nominal' to be same value as 'mEva_flow_nominal', 
type the value in directly here.

3. In the diagram model, double click on 'sou1', update 'm1_nominal' to be same value as 'mCon_flow_nominal', type the value in 
directly here.

mEva_flow_nominal and mCon_flow_nominal can be found in following location in the information or text layer of the dataset: 
ElementsCWModel3.ChillerData.ElectricEIRCarrier_30RB360, select corresponding chiller.

No changes need to be made to the base classes with these models. Modifications can be made directly in the top layer model.
<pre>
<p>
Example that simulates a chiller whose efficiency is computed based on the
condenser entering and evaporator leaving fluid temperature.
A bicubic polynomial is used to compute the chiller part load performance.

Constant pump and fan speeds.
</p>
</html>",     revisions="<html>
<ul>
<li>
December 10, 2018, by Anneka Kang - changed to air cooled chiller, added cooling coil and weabus links.
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(coordinateSystem(extent={{-120,-100},{140,120}})),
        Icon(coordinateSystem(extent={{-120,-100},{140,120}})));
    end ElectricEIRConst;
  annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in
<a href=\"modelica://Buildings.Fluid.Chillers\">
Buildings.Fluid.Chillers</a>.
</p>
</html>"));
  end Task2a;

  package ChillerData
    record ElectricEIRCarrier_30RB360 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =  -1248400,
        COP_nominal =         2.8,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.0536,
        mCon_flow_nominal =   1.2*128.75,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 15.56,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 48.89,
        capFunT =             {0.8995510015,0.0631635527,-0.0007632716,-0.0011664626,-0.0001085415,-0.0004633207},
        EIRFunT =             {0.9617522343,-0.018761025,0.0013138214,-0.0175269675,0.0007280484,-0.0007655058},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "ElectricEIRChiller Carrier 30RB360 1248.4kW/2.8COP" annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
<pre>                 
NOTE: when changing chiller model the following parameters should be updated:
mCHW_flow_nominal should be changed in the text layer of the main model (e.g ElectricEIRConst), this should be the same value as 'mEva_flow_nominal'.
In the diagram model, double click on 'coocoi', update 'm1_flow_nominal' to be same value as 'mEva_flow_nominal', type the value in directly here.
In the diagram model, double click on 'sou1', update 'm1_nominal' to be same value as 'mCon_flow_nominal', type the value in directly here.
<pre>
                 
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller Carrier 30RB360 1248.4kW/2.8COP,  !- Name
    1248400,                  !- Reference Capacity {W}
    2.8,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.0536,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*128.75,                 !- Reference Condenser Water Flow Rate {m3/s}
   
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
    record ElectricEIRChiller_30XA240 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =   -801600,
        COP_nominal =         3.0,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.0345,
        mCon_flow_nominal =   1.2*76,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 10,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 46.11,
        capFunT =             {0.7844608307,0.0496965209,-0.0007704563,0.0062651469,-0.0001580228,-0.0004502762},
        EIRFunT =             {-0.3078657246,0.0607254323,0.000999675,0.0391939439,-0.0000010751,-0.0021290772},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "ElectricEIRChiller Carrier 30XA240 801.6kW/3COP" annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller McQuay WSC 471kW/5.89COP/Vanes,  !- Name
    801600,                  !- Reference Capacity {W}
    3.0,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.0345,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*76,                 !- Reference Condenser Water Flow Rate {m3/s}
    ElectricEIRChiller Carrier 30XA240 801.6kW/3COP CAPFT,  !- Cooling Capacity Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30XA240 801.6kW/3COP EIRFT,  !- Electric Input to Cooling Output Ratio Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30XA240 801.6kW/3COP EIRFPLR,  !- Electric Input to Cooling Output Ratio Function of Part Load Ratio Curve Name
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
    record ElectricEIRChiller_30RB300 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =   -993800,
        COP_nominal =         2.8,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.0426,
        mCon_flow_nominal =   1.2*105.3,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 15.56,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 48.89,
        capFunT =             {0.9060558463,0.0592793701,-0.0004342551,-0.0008732551,-0.0001057317,-0.000504074},
        EIRFunT =             {0.9462638951,-0.0161900368,0.0010820195,-0.0169719483,0.0007127039,-0.0007308545},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "ElectricEIRChiller Carrier 30RB300 993.8kW/2.8COP" annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller Carrier 30RB300 993.8kW/2.8COP,  !- Name
    993800,                  !- Reference Capacity {W}
    2.8,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.0426,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*105.3,                 !- Reference Condenser Water Flow Rate {m3/s}
    ElectricEIRChiller Carrier 30RB300 993.8kW/2.8COP CAPFT,  !- Cooling Capacity Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB300 993.8kW/2.8COP EIRFT,  !- Electric Input to Cooling Output Ratio Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB300 993.8kW/2.8COP EIRFPLR,  !- Electric Input to Cooling Output Ratio Function of Part Load Ratio Curve Name
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
    record ElectricEIRChiller_30RB275 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =   -915000,
        COP_nominal =         2.8,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.0393,
        mCon_flow_nominal =   1.2*93.634,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 15.56,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 48.89,
        capFunT =             {0.900268869,0.0605644181,-0.0006154471,-0.000769599,-0.0001112917,-0.0004645964},
        EIRFunT =             {0.9466655778,-0.0160957707,0.0012206222,-0.0171667889,0.0007215738,-0.0007811794},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "ElectricEIRChiller Carrier 30RB275 915kW/2.8COP"   annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller Carrier 30RB275 993.8kW/2.8COP,  !- Name
    915000,                  !- Reference Capacity {W}
    2.8,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.0393,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*93.634,                 !- Reference Condenser Water Flow Rate {m3/s}
    ElectricEIRChiller Carrier 30RB275 915kW/2.8COP CAPFT,  !- Cooling Capacity Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB275 915kW/2.8COP EIRFT,  !- Electric Input to Cooling Output Ratio Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB275 915kW/2.8COP EIRFPLR,  !- Electric Input to Cooling Output Ratio Function of Part Load Ratio Curve Name
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
    record ElectricEIRChiller_30XA220 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =   -743700,
        COP_nominal =         3.1,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.032,
        mCon_flow_nominal =   1.2*76.1,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 15.56,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 48.89,
        capFunT =             {0.9476404278,0.0425358256,-0.0008745529,-0.0030162359,-0.0000548418,-0.0000909535},
        EIRFunT =             {0.1560148274,0.0147079682,0.0026431064,0.0230803367,0.0001935252,-0.0017368676},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "ElectricEIRChiller Carrier 30XA220 743.7kW/3.1COP" annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller Carrier 30XA220 743.7kW/3.1COP,  !- Name
    743700,                  !- Reference Capacity {W}
    3.1,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.032,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*76.1,                 !- Reference Condenser Water Flow Rate {m3/s}
    ElectricEIRChiller Carrier 30XA220 743.7kW/3.1COP CAPFT,  !- Cooling Capacity Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30XA220 743.7kW/3.1COP EIRFT,  !- Electric Input to Cooling Output Ratio Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30XA220 743.7kW/3.1COP EIRFPLR,  !- Electric Input to Cooling Output Ratio Function of Part Load Ratio Curve Name
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
    record ElectricEIRChiller_30RB210 =
      Buildings.Fluid.Chillers.Data.ElectricEIR.Generic (
        QEva_flow_nominal =   -710000,
        COP_nominal =         2.9,
        PLRMin =              0.10,
        PLRMinUnl =           0.10,
        PLRMax =              1.15,
        mEva_flow_nominal =   1000*0.0305,
        mCon_flow_nominal =   1.2*70.226,
        TEvaLvg_nominal =     273.15 + 6.67,
        TConEnt_nominal =     273.15 + 35,
        TEvaLvgMin =          273.15 + 4.44,
        TEvaLvgMax =          273.15 + 15.56,
        TConEntMin =          273.15 + 29.44,
        TConEntMax =          273.15 + 48.89,
        capFunT =             {0.8728556716,0.067552862,-0.0011909496,-0.0005827711,-0.0001216022,-0.0004005835},
        EIRFunT =             {0.9810690575,-0.0214899832,0.0016475479,-0.0181631771,0.0007433634,-0.000826312},
        EIRFunPLR =           {0.0,1.0,0.0},
        etaMotor =            1.0)
      "c"   annotation (
      defaultComponentName="datChi",
      defaultComponentPrefixes="parameter",
      Documentation(info=
                     "<html>
Performance data for chiller model.
This data corresponds to the following EnergyPlus model:
<pre>
Chiller:Electric:EIR,
    ElectricEIRChiller Carrier 30RB210 710kW/2.9COP,  !- Name
    710000,                  !- Reference Capacity {W}
    3.1,                    !- Reference COP {W/W}
    6.67,                    !- Reference Leaving Chilled Water Temperature {C}
    35,                   !- Reference Entering Condenser Fluid Temperature {C}
    0.0305,                 !- Reference Chilled Water Flow Rate {m3/s}
    1.2*70.226,                 !- Reference Condenser Water Flow Rate {m3/s}
    ElectricEIRChiller Carrier 30RB210 710kW/2.9COP CAPFT,  !- Cooling Capacity Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB210 710kW/2.9COP EIRFT,  !- Electric Input to Cooling Output Ratio Function of Temperature Curve Name
    ElectricEIRChiller Carrier 30RB210 710kW/2.9COP EIRFPLR,  !- Electric Input to Cooling Output Ratio Function of Part Load Ratio Curve Name
    0.10,                    !- Minimum Part Load Ratio
    1.15,                    !- Maximum Part Load Ratio
    1.0,                     !- Optimum Part Load Ratio
    0.10,                    !- Minimum Unloading Ratio
    Chilled Water Side Inlet Node,  !- Chilled Water Inlet Node Name
    Chilled Water Side Outlet Node,  !- Chilled Water Outlet Node Name
    Condenser Side Inlet Node,  !- Condenser Inlet Node Name
    Condenser Side Outlet Node,  !- Condenser Outlet Node Name
    WaterCooled,             !- Condenser Type
    ,                        !- Condenser Fan Power Ratio {W/W}
    1.0,                     !- Compressor Motor Efficiency
    2.0,                     !- Leaving Chilled Water Lower Temperature Limit {C}
    ConstantFlow,            !- Chiller Flow Mode
    0.0;                     !- Design Heat Recovery Water Flow Rate {m3/s}
</pre>
</html>"));
  end ChillerData;

  package Task2b

    model ElectricEIRCW "Electric chiller based on the DOE-2.1 model"
      extends Buildings.Fluid.Chillers.BaseClasses.PartialElectric(
      final QEva_flow_nominal = per.QEva_flow_nominal,
      final COP_nominal= per.COP_nominal,
      final PLRMax= per.PLRMax,
      final PLRMinUnl= per.PLRMinUnl,
      final PLRMin= per.PLRMin,
      final etaMotor= per.etaMotor,
      final mEva_flow_nominal= per.mEva_flow_nominal,
      final mCon_flow_nominal= per.mCon_flow_nominal,
      final TEvaLvg_nominal= per.TEvaLvg_nominal);

      parameter Task2_3.ChillerData.ElectricEIRCarrier_30RB360 per
        "Performance data" annotation (choicesAllMatching=true, Placement(
            transformation(extent={{40,80},{60,100}})));

    protected
      final parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC
        TConEnt_nominal_degC=
        Modelica.SIunits.Conversions.to_degC(per.TConEnt_nominal)
        "Temperature of fluid entering condenser at nominal condition";

      Modelica.SIunits.Conversions.NonSIunits.Temperature_degC TConEnt_degC
        "Temperature of fluid entering condenser";
    initial equation
      // Verify correctness of performance curves, and write warning if error is bigger than 10%
      Buildings.Fluid.Chillers.BaseClasses.warnIfPerformanceOutOfBounds(
         Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT,
         x1=TEvaLvg_nominal_degC, x2=TConEnt_nominal_degC),
         "Capacity as function of temperature ",
         "per.capFunT");
    equation
      TConEnt_degC=Modelica.SIunits.Conversions.to_degC(TConEnt);

      if on then
        // Compute the chiller capacity fraction, using a biquadratic curve.
        // Since the regression for capacity can have negative values (for unreasonable temperatures),
        // we constrain its return value to be non-negative. This prevents the solver to pick the
        // unrealistic solution.
        capFunT = Buildings.Utilities.Math.Functions.smoothMax(
           x1 = 1E-6,
           x2 = Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT, x1=TEvaLvg_degC, x2=TConEnt_degC),
           deltaX = 1E-7);
    /*    assert(capFunT > 0.1, "Error: Received capFunT = " + String(capFunT)  + ".\n"
           + "Coefficient for polynomial seem to be not valid for the encountered temperature range.\n"
           + "Temperatures are TConEnt_degC = " + String(TConEnt_degC) + " degC\n"
           + "                 TEvaLvg_degC = " + String(TEvaLvg_degC) + " degC");
*/
        // Chiller energy input ratio biquadratic curve.
        EIRFunT = Buildings.Utilities.Math.Functions.biquadratic(a=per.EIRFunT, x1=TEvaLvg_degC, x2=TConEnt_degC);
        // Chiller energy input ratio quadratic curve
        EIRFunPLR   = per.EIRFunPLR[1]+per.EIRFunPLR[2]*PLR2+per.EIRFunPLR[3]*PLR2^2;
      else
        capFunT   = 0;
        EIRFunT   = 0;
        EIRFunPLR = 0;
      end if;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                       graphics={
            Rectangle(
              extent={{-99,-54},{102,-66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,-66},{0,-54}},
              lineColor={0,0,127},
              pattern=LinePattern.None,
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-104,66},{98,54}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-2,54},{98,66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-44,52},{-40,12}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-56,70},{58,52}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-42,2},{-52,12},{-32,12},{-42,2}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-44,-10},{-40,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{38,52},{42,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-56,-50},{58,-68}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{18,24},{62,-18}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,24},{22,-8},{58,-8},{40,24}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
    defaultComponentName="chi",
    Documentation(info="<html>
<p>
Model of an electric chiller, based on the DOE-2.1 chiller model and
the EnergyPlus chiller model <code>Chiller:Electric:EIR</code>.
</p>
<p> This model uses three functions to predict capacity and power consumption:
</p>
<ul>
<li>
A biquadratic function is used to predict cooling capacity as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
<li>
A quadratic functions is used to predict power input to cooling capacity ratio with respect to the part load ratio.
</li>
<li>
A biquadratic functions is used to predict power input to cooling capacity ratio as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
</ul>
<p>
These curves are stored in the data record <code>per</code> and are available from
<a href=\"Buildings.Fluid.Chillers.Data.ElectricEIR\">
Buildings.Fluid.Chillers.Data.ElectricEIR</a>.
Additional performance curves can be developed using
two available techniques (Hydeman and Gillespie, 2002). The first technique is called the
Least-squares Linear Regression method and is used when sufficient performance data exist
to employ standard least-square linear regression techniques. The second technique is called
Reference Curve Method and is used when insufficient performance data exist to apply linear
regression techniques. A detailed description of both techniques can be found in
Hydeman and Gillespie (2002).
</p>
<p>
The model takes as an input the set point for the leaving chilled water temperature,
which is met if the chiller has sufficient capacity.
Thus, the model has a built-in, ideal temperature control.
The model has three tests on the part load ratio and the cycling ratio:
</p>
<ol>
<li>
The test<pre>
  PLR1 =min(QEva_flow_set/QEva_flow_ava, per.PLRMax);
</pre>
ensures that the chiller capacity does not exceed the chiller capacity specified
by the parameter <code>per.PLRMax</code>.
</li>
<li>
The test <pre>
  CR = min(PLR1/per.PRLMin, 1.0);
</pre>
computes a cycling ratio. This ratio expresses the fraction of time
that a chiller would run if it were to cycle because its load is smaller than the
minimal load at which it can operate.
Note that this model continuously operates even if the part load ratio is below the minimum part load ratio.
Its leaving evaporator and condenser temperature can therefore be considered as an
average temperature between the modes where the compressor is off and on.
</li>
<li>
The test <pre>
  PLR2 = max(per.PLRMinUnl, PLR1);
</pre>
computes the part load ratio of the compressor.
The assumption is that for a part load ratio below <code>per.PLRMinUnl</code>,
the chiller uses hot gas bypass to reduce the capacity, while the compressor
power draw does not change.
</li>
</ol>
<p>
The electric power only contains the power for the compressor, but not any power for pumps or fans.
</p>
<p>
The model can be parametrized to compute a transient
or steady-state response.
The transient response of the boiler is computed using a first
order differential equation for the evaporator and condenser fluid volumes.
The chiller outlet temperatures are equal to the temperatures of these lumped volumes.
</p>
<h4>References</h4>
<ul>
<li>
Hydeman, M. and K.L. Gillespie. 2002. Tools and Techniques to Calibrate Electric Chiller
Component Models. <i>ASHRAE Transactions</i>, AC-02-9-1.
</li>
</ul>
</html>",
    revisions="<html>
<ul>
<li>
March 12, 2015, by Michael Wetter:<br/>
Refactored model to make it once continuously differentiable.
This is for issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/373\">373</a>.
</li>
<li>
Jan. 9, 2011, by Michael Wetter:<br/>
Added input signal to switch chiller off.
</li>
<li>
Sep. 8, 2010, by Michael Wetter:<br/>
Revised model and included it in the Buildings library.
</li>
<li>
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"));
    end ElectricEIRCW;

    model ElectricEIRVP "Chiller Electric EIR with Water - Air Cooling Coil"
      extends Modelica.Icons.Example;
      extends Task2_3.Task2c.BaseClasses.PartialElectric(
        P_nominal=-per.QEva_flow_nominal/per.COP_nominal,
        mEva_flow_nominal=per.mEva_flow_nominal,
        mCon_flow_nominal=per.mCon_flow_nominal,
        sou1(
          redeclare package Medium = Medium1,
          m_flow=1.2*76.1,
          nPorts=1),
        TSetChi(k=TsetChi),
        sin1(p=101325));
      replaceable package MediumA = Buildings.Media.Air "Medium model";
      replaceable package MediumW = Buildings.Media.Water "Medium model";
      parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal=16 "Nominal mass flow rate at fan";
      parameter Modelica.SIunits.MassFlowRate mCHW_flow_nominal=32 "Nominal mass flow rate at chilled water";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal = mCHW_flow_nominal
        "Nominal mass flow rate";
      parameter Modelica.SIunits.Temperature TsetChi=280.15 "Chilled water set point";
      parameter Modelica.SIunits.Temperature TAirSet=286.15 "Cooled air set point";
      parameter Task2_3.ChillerData.ElectricEIRChiller_30XA220 per
        "Chiller performance data"
        annotation (Placement(transformation(extent={{70,76},{94,100}})));

      Task2_3.Task2c.ElectricEIRCW Chiller(
        redeclare package Medium2 = Medium2,
        per=per,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp1_nominal=6000,
        dp2_nominal=6000,
        redeclare package Medium1 = Medium1) "Chiller model"
        annotation (Placement(transformation(extent={{-2,36},{18,56}})));

      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaData(filNam=
            "C:/Users/BT/OneDrive - University College London/Engineered Environmental Elements/3E COURSEWORK/modelica-buildings-master/Task/ESP_Barcelona.081810_SWEC.mos")
        annotation (Placement(transformation(extent={{138,-72},{118,-52}})));
      Buildings.Fluid.Sources.FixedBoundary sinA(
        nPorts=1,
        redeclare package Medium = MediumA,
        use_T=true)
        annotation (Placement(transformation(extent={{-90,-72},{-70,-52}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TAirOut(
                               redeclare package Medium = MediumA, m_flow_nominal=
            mAir_flow_nominal) "Temperature of air leaving cooling coil"
                                                  annotation (Placement(
            transformation(
            extent={{7,6},{-7,-6}},
            rotation=0,
            origin={-41,-62})));
      Buildings.Fluid.Sources.Outside Weather_Data(nPorts=1, redeclare package
          Medium = MediumA)
        annotation (Placement(transformation(extent={{94,-72},{74,-52}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow Fan(
        redeclare package Medium = MediumA,
        m_flow_nominal=mAir_flow_nominal,
        nominalValuesDefineDefaultPressureCurve=true,
        dp_nominal=600)       annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={58,-62})));
      Buildings.Fluid.Storage.ExpansionVessel expVesChi(V_start=1, redeclare
          package Medium = Medium2)
        annotation (Placement(transformation(extent={{-10,-40},{4,-25}})));
      Modelica.Blocks.Sources.Constant mFanFlo(k=mAir_flow_nominal)
        "Mass flow rate of fan" annotation (Placement(transformation(extent={{8,-92},
                {20,-80}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_OUT(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water leaving chiller " annotation (Placement(
            transformation(
            extent={{-6,6},{6,-6}},
            rotation=270,
            origin={-32,-28})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_IN(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water returning to chiller " annotation (Placement(
            transformation(
            extent={{-4,4},{4,-4}},
            rotation=90,
            origin={42,-42})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TAirFan(redeclare package
          Medium =
            MediumA, m_flow_nominal=mAir_flow_nominal)
        "Temperature of air leaving fan" annotation (Placement(transformation(
            extent={{5,6},{-5,-6}},
            rotation=0,
            origin={39,-62})));
      Modelica.Blocks.Continuous.Integrator integratorChi
        annotation (Placement(transformation(extent={{38,70},{52,84}})));
      Modelica.Blocks.Continuous.Integrator integratorPump
        annotation (Placement(transformation(extent={{84,24},{98,38}})));
      Modelica.Blocks.Continuous.Integrator integratorFan
        annotation (Placement(transformation(extent={{-4,-82},{-18,-68}})));
      Buildings.Fluid.HeatExchangers.WetCoilCounterFlow CoolingCoil(
        redeclare package Medium1 = Medium2,
        redeclare package Medium2 = MediumA,
        UA_nominal=mAir_flow_nominal*1006*5,
        dp1_nominal=1000 + 89580,
        dp2_nominal=249*3,
        m2_flow_nominal=16,
        m1_flow_nominal=32)
        annotation (Placement(transformation(extent={{6,-66},{26,-46}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow ChillerPump(
        redeclare package Medium = Medium2,
        nominalValuesDefineDefaultPressureCurve=true,
        m_flow_nominal=mCHW_flow_nominal,
        dp_nominal=179352)     annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={42,22})));
      Buildings.Fluid.FixedResistances.Junction splitter(
        from_dp=true,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        linearized=true,
        dp_nominal(each displayUnit="Pa") = {0,0,0},
        m_flow_nominal=mCHW_flow_nominal*{1,1,1},
        redeclare package Medium = Medium2) "Splitter for bypass" annotation (
          Placement(transformation(
            extent={{-6,6},{6,-6}},
            rotation=90,
            origin={42,-4})));
      Buildings.Fluid.FixedResistances.Junction splitter2(
        from_dp=true,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        linearized=true,
        dp_nominal(each displayUnit="Pa") = {0,0,0},
        redeclare package Medium = Medium2,
        m_flow_nominal=mCHW_flow_nominal*{1,1,1}) "Splitter for bypass"
        annotation (Placement(transformation(
            extent={{-6,6},{6,-6}},
            rotation=270,
            origin={-32,-4})));
      Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage ModulatingValve(
        CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
        dpValve_nominal=6000,
        from_dp=true,
        riseTime=10,
        use_inputFilter=false,
        redeclare package Medium = Medium2,
        m_flow_nominal=mCHW_flow_nominal)
        "variable valve to control chilled water loop flow rate" annotation (
          Placement(transformation(
            extent={{-6,6},{6,-6}},
            rotation=90,
            origin={42,-26})));
      Modelica.Blocks.Sources.Constant TAirOutSet(k=TAirSet) "TAirOut set point "
        annotation (Placement(transformation(extent={{134,-38},{118,-22}})));
      Modelica.Blocks.Sources.Constant mPumpFlo1(k=mCHW_flow_nominal)
        "Mass flow rate of pump"
        annotation (Placement(transformation(extent={{98,-2},{84,12}})));
      Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valBypass(
        CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
        dpValve_nominal=6000,
        from_dp=true,
        riseTime=10,
        use_inputFilter=false,
        redeclare package Medium = Medium2,
        m_flow_nominal=mCHW_flow_nominal)
        "bypass valve, half open to maintain correct pressure"    annotation (
          Placement(transformation(
            extent={{6,6},{-6,-6}},
            rotation=180,
            origin={4,-4})));
      Modelica.Blocks.Sources.Constant mPumpFlo2(k=0.5)
        "Mass flow rate of pump"
        annotation (Placement(transformation(extent={{-20,12},{-6,26}})));
      Buildings.Controls.Continuous.LimPID preHeaCoiCon(
        Td=60,
        initType=Modelica.Blocks.Types.InitPID.InitialState,
        strict=true,
        yMax=1,
        yMin=0,
        xi_start=0,
        k=100,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        wp=0,
        reverseAction=true,
        Ti(displayUnit="min") = 1)
               "Controller for pre-heating coil"
        annotation (Placement(transformation(extent={{96,-36},{84,-24}})));
    equation
      connect(Chiller.port_b1, res1.port_a) annotation (Line(
          points={{18,52},{34,52}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(Chiller.on, greaterThreshold.y) annotation (Line(
          points={{-4,49},{-10,49},{-10,86},{-25,86}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(TSetChi.y, Chiller.TSet) annotation (Line(points={{-75,2},{-50,2},
              {-50,43},{-4,43}}, color={0,0,127}));
      connect(TAirOut.port_b,sinA. ports[1])
        annotation (Line(points={{-48,-62},{-70,-62}}, color={0,127,255}));
      connect(Weather_Data.ports[1], Fan.port_a)
        annotation (Line(points={{74,-62},{68,-62}}, color={0,127,255}));
      connect(mFanFlo.y, Fan.m_flow_in)
        annotation (Line(points={{20.6,-86},{58,-86},{58,-74}}, color={0,0,127}));

      connect(Weather_Data.weaBus, weaData.weaBus) annotation (Line(
          points={{94,-61.8},{100,-61.8},{100,-62},{118,-62}},
          color={255,204,51},
          thickness=0.5));

      connect(Fan.port_b, TAirFan.port_a)
        annotation (Line(points={{48,-62},{44,-62}}, color={0,127,255}));
      connect(Chiller.P, integratorChi.u) annotation (Line(points={{19,55},{20,
              55},{20,77},{36.6,77}}, color={0,0,127}));
      connect(Fan.P, integratorFan.u) annotation (Line(points={{47,-71},{44,-71},{44,
              -75},{-2.6,-75}},   color={0,0,127}));
      connect(CoolingCoil.port_b2, TAirOut.port_a)
        annotation (Line(points={{6,-62},{-34,-62}}, color={0,127,255}));
      connect(TAirFan.port_b, CoolingCoil.port_a2)
        annotation (Line(points={{34,-62},{26,-62}}, color={0,127,255}));
      connect(TCHW_IN.port_a, CoolingCoil.port_b1) annotation (Line(points={{42,
              -46},{42,-50},{26,-50}}, color={0,127,255}));
      connect(CoolingCoil.port_a1, TCHW_OUT.port_b) annotation (Line(points={{6,
              -50},{-32,-50},{-32,-34}}, color={0,127,255}));
      connect(expVesChi.port_a, TCHW_OUT.port_b) annotation (Line(points={{-3,-40},
              {-3,-46},{-32,-46},{-32,-34}}, color={0,127,255}));
      connect(ChillerPump.port_b, Chiller.port_a2)
        annotation (Line(points={{42,32},{42,40},{18,40}}, color={0,127,255}));
      connect(ChillerPump.P, integratorPump.u) annotation (Line(points={{51,33},{48,
              33},{48,31},{82.6,31}},     color={0,0,127}));
      connect(splitter.port_2, ChillerPump.port_a)
        annotation (Line(points={{42,2},{42,12}}, color={0,127,255}));
      connect(TCHW_OUT.port_a, splitter2.port_2)
        annotation (Line(points={{-32,-22},{-32,-10}}, color={0,127,255}));
      connect(Chiller.port_b2, splitter2.port_1) annotation (Line(points={{-2,
              40},{-32,40},{-32,2}}, color={0,127,255}));
      connect(TCHW_IN.port_b, ModulatingValve.port_a)
        annotation (Line(points={{42,-38},{42,-32}}, color={0,127,255}));
      connect(ModulatingValve.port_b, splitter.port_1)
        annotation (Line(points={{42,-20},{42,-10}}, color={0,127,255}));
      connect(mPumpFlo1.y, ChillerPump.m_flow_in) annotation (Line(points={{83.3,5},
              {68,5},{68,22},{54,22}},      color={0,0,127}));
      connect(splitter.port_3, valBypass.port_b)
        annotation (Line(points={{36,-4},{10,-4}}, color={0,127,255}));
      connect(valBypass.port_a, splitter2.port_3)
        annotation (Line(points={{-2,-4},{-26,-4}}, color={0,127,255}));
      connect(mPumpFlo2.y, valBypass.y)
        annotation (Line(points={{-5.3,19},{4,19},{4,3.2}},color={0,0,127}));
      connect(preHeaCoiCon.y, ModulatingValve.y) annotation (Line(points={{83.4,
              -30},{66,-30},{66,-26},{49.2,-26}}, color={0,0,127}));
      connect(TAirOutSet.y, preHeaCoiCon.u_s)
        annotation (Line(points={{117.2,-30},{97.2,-30}}, color={0,0,127}));
      connect(TAirOut.T, preHeaCoiCon.u_m) annotation (Line(points={{-41,-68.6},{-41,
              -98},{102,-98},{102,-44},{90,-44},{90,-37.2}}, color={0,0,127}));
      connect(sou1.weaBus, weaData.weaBus) annotation (Line(
          points={{-90,52.2},{-100,52.2},{-100,-104},{110,-104},{110,-62},{118,-62}},
          color={255,204,51},
          thickness=0.5));

      connect(sou1.ports[1], Chiller.port_a1)
        annotation (Line(points={{-70,52},{-2,52}}, color={0,127,255}));
      annotation (
    experiment(
          StartTime=20044800,
          StopTime=20649600,
          Interval=3600,
          Tolerance=0.001),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Chillers/Examples/ElectricEIR.mos"
            "Simulate and plot"),
        Documentation(info="<html>
        <pre>                 
NOTE: when changing chiller model the following parameters should be updated:
1. mCHW_flow_nominal should be changed in the text layer of the main model (e.g ElectricEIRConst), this should be the same value as 
'mEva_flow_nominal'.

2. In the diagram model, double click on 'coocoi', update 'm1_flow_nominal' to be same value as 'mEva_flow_nominal', 
type the value in directly here.

3. In the diagram model, double click on 'sou1', update 'm1_nominal' to be same value as 'mCon_flow_nominal', type the value in 
directly here.

mEva_flow_nominal and mCon_flow_nominal can be found in following location in the information or text layer of the dataset: 
ElementsCWModel3.ChillerData.ElectricEIRCarrier_30RB360, select corresponding chiller.

No changes need to be made to the base classes with these models. Modifications can be made directly in the top layer model.
<pre>
<p>
Example that simulates a chiller whose efficiency is computed based on the
condenser entering and evaporator leaving fluid temperature.
A bicubic polynomial is used to compute the chiller part load performance.

The flow rate in the chilled water loop is modulated by a valve controlled by a PI controller to meet the cooled air set point.
</p>
</html>",     revisions="<html>
<ul>
<li>
December 10, 2018, by Anneka Kang - changed to air cooled chiller, added cooling coil, PI control and weabus links.
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(coordinateSystem(extent={{-120,-120},{140,120}})),
        Icon(coordinateSystem(extent={{-120,-120},{140,120}})));
    end ElectricEIRVP;

  end Task2b;

  package Task2c
    model ElectricEIRVFan
      "Chiller Electric EIR with Water - Air Cooling Coil"
      extends Modelica.Icons.Example;
      extends Task2_3.Task2c.BaseClasses.PartialElectric(
        P_nominal=-per.QEva_flow_nominal/per.COP_nominal,
        mEva_flow_nominal=per.mEva_flow_nominal,
        mCon_flow_nominal=per.mCon_flow_nominal,
        TSetChi(k=TsetChi),
        sou1(redeclare package Medium = Medium1, m_flow=1.2*76.1));

      replaceable package MediumA = Buildings.Media.Air "Medium model";
      replaceable package MediumW = Buildings.Media.Water "Medium model";
      parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal=16 "Nominal mass flow rate at fan";
      parameter Modelica.SIunits.MassFlowRate mCHW_flow_nominal=32 "Nominal mass flow rate at chilled water";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal = mCHW_flow_nominal
        "Nominal mass flow rate";
      parameter Modelica.SIunits.Temperature TsetChi=280.15 "Chilled water set point";
      parameter Modelica.SIunits.Temperature TAirSet=286.15 "Cooled air set point";
       parameter Real minSpeFan = 0.01 "Min fan speed load";
      parameter Task2_3.ChillerData.ElectricEIRChiller_30XA220 per
        "Chiller performance data"
        annotation (Placement(transformation(extent={{70,76},{94,100}})));

      Task2_3.Task2b.ElectricEIRCW Chiller(
        redeclare package Medium2 = Medium2,
        per=per,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp1_nominal=6000,
        dp2_nominal=6000,
        redeclare package Medium1 = Medium1) "Chiller model"
        annotation (Placement(transformation(extent={{-2,36},{18,56}})));

      Buildings.Fluid.HeatExchangers.DryCoilCounterFlow CoolingCoil(
        redeclare package Medium2 = MediumA,
        m1_flow(start=mCHW_flow_nominal),
        m2_flow(start=mAir_flow_nominal),
        dp2_nominal=249*3,
        UA_nominal=mAir_flow_nominal*1006*5,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp1_nominal(displayUnit="Pa") = 1000 + 89580,
        redeclare package Medium1 = Medium2,
        m2_flow_nominal=16,
        m1_flow_nominal=32) "Cooling coil" annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={-2,-50})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaData(filNam=
            "C:/Users/BT/OneDrive - University College London/Engineered Environmental Elements/3E COURSEWORK/modelica-buildings-master/Task/ESP_Barcelona.081810_SWEC.mos")
        annotation (Placement(transformation(extent={{130,-84},{110,-64}})));
      Modelica.Blocks.Sources.Constant mPumpFlo1(k=mCHW_flow_nominal)
        "Mass flow rate of pump"
        annotation (Placement(transformation(extent={{90,20},{74,36}})));
      Buildings.Fluid.Sources.FixedBoundary sinA(
        nPorts=1,
        redeclare package Medium = MediumA,
        use_T=true)
        annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TAirOut(m_flow_nominal=
            mAir_flow_nominal, redeclare package Medium = MediumA)
        "Temperature of air leaving cooling coil" annotation (Placement(
            transformation(
            extent={{10,10},{-10,-10}},
            rotation=0,
            origin={-40,-56})));
      Buildings.Fluid.Sources.Outside Weather_Data(redeclare package Medium =
            MediumA, nPorts=1)
        annotation (Placement(transformation(extent={{96,-66},{76,-46}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow ChillerPump(
        redeclare package Medium = Medium2,
        nominalValuesDefineDefaultPressureCurve=true,
        m_flow_nominal=mCHW_flow_nominal,
        dp_nominal=179352)     annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={38,10})));
      Buildings.Fluid.Storage.ExpansionVessel expVesChi(V_start=1, redeclare
          package Medium = Medium2)
        annotation (Placement(transformation(extent={{-26,-34},{-12,-21}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_OUT(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water leaving chiller " annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-32,4})));
      Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_IN(m_flow_nominal=
            mCHW_flow_nominal, redeclare package Medium = Medium2)
        "Temperature of water returning to chiller " annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={38,-20})));
      Buildings.Controls.Continuous.LimPID
                                 conPID(
        Td=1,
        k=0.5,
        yMin=minSpeFan,
        yMax=50,
        controllerType=Modelica.Blocks.Types.SimpleController.PID,
        Ti=200,
        reverseAction=false)
              annotation (Placement(transformation(extent={{20,-90},{32,-78}})));
      Modelica.Blocks.Sources.Constant TAirOutSet(k=TAirSet) "TAirOut set point "
        annotation (Placement(transformation(extent={{-2,-90},{10,-78}})));
      Buildings.Fluid.Movers.SpeedControlled_y Fan(
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        addPowerToMedium=false,
        use_inputFilter=true,
        redeclare package Medium = MediumA,
        per(pressure(V_flow=mAir_flow_nominal*{0,1,2}/1.2, dp=500*{2,1,0})))
        "Supply air fan"
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=0,
            origin={52,-56})));
      Modelica.Blocks.Continuous.Integrator integratorPump
        annotation (Placement(transformation(extent={{76,-6},{90,8}})));
      Modelica.Blocks.Continuous.Integrator integratorFan
        annotation (Placement(transformation(extent={{-10,-74},{-20,-64}})));
      Modelica.Blocks.Continuous.Integrator integratorChi
        annotation (Placement(transformation(extent={{38,70},{52,84}})));
    equation
      connect(Chiller.port_b1, res1.port_a) annotation (Line(
          points={{18,52},{34,52}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(Chiller.on, greaterThreshold.y) annotation (Line(
          points={{-4,49},{-10,49},{-10,86},{-25,86}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(TSetChi.y, Chiller.TSet) annotation (Line(points={{-75,2},{-50,2},
              {-50,43},{-4,43}}, color={0,0,127}));
      connect(CoolingCoil.port_b2, TAirOut.port_a)
        annotation (Line(points={{-12,-56},{-30,-56}}, color={0,127,255}));
      connect(TAirOut.port_b,sinA. ports[1])
        annotation (Line(points={{-50,-56},{-70,-56}}, color={0,127,255}));
      connect(Weather_Data.weaBus, weaData.weaBus) annotation (Line(
          points={{96,-55.8},{96,-74},{110,-74}},
          color={255,204,51},
          thickness=0.5));
      connect(CoolingCoil.port_a1, expVesChi.port_a) annotation (Line(points={{
              -12,-44},{-19,-44},{-19,-34}}, color={0,127,255}));
      connect(ChillerPump.port_b, Chiller.port_a2)
        annotation (Line(points={{38,20},{38,40},{18,40}}, color={0,127,255}));
      connect(ChillerPump.m_flow_in, mPumpFlo1.y) annotation (Line(points={{26,10},{
              16,10},{16,28},{73.2,28}}, color={0,0,127}));
      connect(CoolingCoil.port_a1, TCHW_OUT.port_b) annotation (Line(points={{-12,
              -44},{-32,-44},{-32,-6}}, color={0,127,255}));
      connect(TCHW_OUT.port_a, Chiller.port_b2) annotation (Line(points={{-32,
              14},{-32,40},{-2,40}}, color={0,127,255}));
      connect(CoolingCoil.port_b1, TCHW_IN.port_a) annotation (Line(points={{8,
              -44},{38,-44},{38,-30}}, color={0,127,255}));
      connect(TCHW_IN.port_b, ChillerPump.port_a)
        annotation (Line(points={{38,-10},{38,0}},   color={0,127,255}));
      connect(TAirOutSet.y, conPID.u_s)
        annotation (Line(points={{10.6,-84},{18.8,-84}}, color={0,0,127}));
      connect(TAirOut.T, conPID.u_m) annotation (Line(points={{-40,-67},{-40,-98},{26,
              -98},{26,-91.2}},    color={0,0,127}));
      connect(Fan.port_b, CoolingCoil.port_a2)
        annotation (Line(points={{42,-56},{8,-56}}, color={0,127,255}));
      connect(Fan.port_a, Weather_Data.ports[1])
        annotation (Line(points={{62,-56},{76,-56}}, color={0,127,255}));
      connect(conPID.y,Fan. y) annotation (Line(points={{32.6,-84},{52,-84},{52,-68}},
                                   color={0,0,127}));
      connect(sou1.weaBus, weaData.weaBus) annotation (Line(
          points={{-90,52.2},{-110,52.2},{-110,-102},{102,-102},{102,-74},{110,-74}},
          color={255,204,51},
          thickness=0.5));

      connect(sou1.ports[1], Chiller.port_a1)
        annotation (Line(points={{-70,52},{-2,52}}, color={0,127,255}));
      connect(ChillerPump.P, integratorPump.u) annotation (Line(points={{29,21},{29,
              24},{62,24},{62,1},{74.6,1}}, color={0,0,127}));
      connect(Fan.P, integratorFan.u) annotation (Line(points={{41,-65},{2,-65},{2,-69},
              {-9,-69}}, color={0,0,127}));
      connect(Chiller.P, integratorChi.u) annotation (Line(points={{19,55},{26,
              55},{26,77},{36.6,77}}, color={0,0,127}));
      annotation (
    experiment(
          StartTime=20044800,
          StopTime=20649600,
          Interval=3600,
          Tolerance=1e-06),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Chillers/Examples/ElectricEIR.mos"
            "Simulate and plot"),
        Documentation(info="<html>
      <pre>                 
NOTE: when changing chiller model the following parameters should be updated:
1. mCHW_flow_nominal should be changed in the text layer of the main model (e.g ElectricEIRConst), this should be the same value as 
'mEva_flow_nominal'.

2. In the diagram model, double click on 'coocoi', update 'm1_flow_nominal' to be same value as 'mEva_flow_nominal', 
type the value in directly here.

3. In the diagram model, double click on 'sou1', update 'm1_nominal' to be same value as 'mCon_flow_nominal', type the value in 
directly here.

mEva_flow_nominal and mCon_flow_nominal can be found in following location in the information or text layer of the dataset: 
ElementsCWModel3.ChillerData.ElectricEIRCarrier_30RB360, select corresponding chiller.

No changes need to be made to the base classes with these models. Modifications can be made directly in the top layer model.
<pre>

<p>
Example that simulates a chiller whose efficiency is computed based on the
condenser entering and evaporator leaving fluid temperature.
A bicubic polynomial is used to compute the chiller part load performance.

This model fan speed is controlled by a PI controller and modulated to meet the set point of the cooled air on the outlet of the cooling coil.
</p>
</html>",     revisions="<html>
<ul>
<li>
December 10, 2018, by Anneka Kang - changed to air cooled chiller, PID control, added cooling coil and weabus links.
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"),
        Diagram(coordinateSystem(extent={{-120,-120},{120,120}})),
        Icon(coordinateSystem(extent={{-120,-120},{120,120}})));
    end ElectricEIRVFan;

    package BaseClasses
      "Package with base classes for Buildings.Fluid.Chillers.Examples"
      extends Modelica.Icons.BasesPackage;
      partial model PartialElectric
        "Base class for test model of chiller electric EIR"
       package Medium1 = Buildings.Media.Air "Medium model";
       package Medium2 = Buildings.Media.Water "Medium model";

        parameter Modelica.SIunits.Power P_nominal
          "Nominal compressor power (at y=1)";
        parameter Modelica.SIunits.TemperatureDifference dTEva_nominal=10
          "Temperature difference evaporator inlet-outlet";

        parameter Real COPc_nominal = 3 "Chiller COP";
        parameter Modelica.SIunits.MassFlowRate mEva_flow_nominal
          "Nominal mass flow rate at evaporator";
        parameter Modelica.SIunits.MassFlowRate mCon_flow_nominal
          "Nominal mass flow rate at condenser";

        Buildings.Fluid.Sources.FixedBoundary sin1(
          redeclare package Medium = Medium1,
          nPorts=1)                           annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              origin={80,52})));
        Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=0.5)
          annotation (Placement(transformation(extent={{-46,76},{-26,96}})));
        Buildings.Fluid.FixedResistances.PressureDrop res1(
          redeclare package Medium = Medium1,
          m_flow_nominal=mCon_flow_nominal,
          dp_nominal=6000) "Flow resistance"
          annotation (Placement(transformation(extent={{34,42},{54,62}})));
        Modelica.Blocks.Sources.Constant TSetChi(k=273.15 + 7)
          "Chiller Water Leaving Temperature Set Point"
          annotation (Placement(transformation(extent={{-96,-8},{-76,12}})));
        Modelica.Blocks.Sources.Constant ChillerON(k=1) "Chiller ON"
          annotation (Placement(transformation(extent={{-98,76},{-78,96}})));
        Buildings.Fluid.Sources.MassFlowSource_WeatherData sou1(nPorts=1,
            redeclare package Medium = Medium1)
          annotation (Placement(transformation(extent={{-90,42},{-70,62}})));
      equation

        connect(res1.port_b, sin1.ports[1]) annotation (Line(
            points={{54,52},{70,52}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(ChillerON.y, greaterThreshold.u)
          annotation (Line(points={{-77,86},{-48,86}}, color={0,0,127}));
      end PartialElectric;
    annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Chillers.Examples\">Buildings.Fluid.Chillers.Examples</a>.
</p>
</html>"));
    end BaseClasses;

    model ElectricEIRCW "Electric chiller based on the DOE-2.1 model"
      extends Task2_3.Task2c.BaseClassesChiller.PartialElectric(
        final QEva_flow_nominal=per.QEva_flow_nominal,
        final COP_nominal=per.COP_nominal,
        final PLRMax=per.PLRMax,
        final PLRMinUnl=per.PLRMinUnl,
        final PLRMin=per.PLRMin,
        final etaMotor=per.etaMotor,
        final mEva_flow_nominal=per.mEva_flow_nominal,
        final mCon_flow_nominal=per.mCon_flow_nominal,
        final TEvaLvg_nominal=per.TEvaLvg_nominal);

      parameter Task2_3.ChillerData.ElectricEIRCarrier_30RB360 per
        "Performance data" annotation (choicesAllMatching=true, Placement(
            transformation(extent={{40,80},{60,100}})));

    protected
      final parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC
        TConEnt_nominal_degC=
        Modelica.SIunits.Conversions.to_degC(per.TConEnt_nominal)
        "Temperature of fluid entering condenser at nominal condition";

      Modelica.SIunits.Conversions.NonSIunits.Temperature_degC TConEnt_degC
        "Temperature of fluid entering condenser";
    initial equation
      // Verify correctness of performance curves, and write warning if error is bigger than 10%
      Buildings.Fluid.Chillers.BaseClasses.warnIfPerformanceOutOfBounds(
         Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT,
         x1=TEvaLvg_nominal_degC, x2=TConEnt_nominal_degC),
         "Capacity as function of temperature ",
         "per.capFunT");
    equation
      TConEnt_degC=Modelica.SIunits.Conversions.to_degC(TConEnt);

      if on then
        // Compute the chiller capacity fraction, using a biquadratic curve.
        // Since the regression for capacity can have negative values (for unreasonable temperatures),
        // we constrain its return value to be non-negative. This prevents the solver to pick the
        // unrealistic solution.
        capFunT = Buildings.Utilities.Math.Functions.smoothMax(
           x1 = 1E-6,
           x2 = Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT, x1=TEvaLvg_degC, x2=TConEnt_degC),
           deltaX = 1E-7);
    /*    assert(capFunT > 0.1, "Error: Received capFunT = " + String(capFunT)  + ".\n"
           + "Coefficient for polynomial seem to be not valid for the encountered temperature range.\n"
           + "Temperatures are TConEnt_degC = " + String(TConEnt_degC) + " degC\n"
           + "                 TEvaLvg_degC = " + String(TEvaLvg_degC) + " degC");
*/
        // Chiller energy input ratio biquadratic curve.
        EIRFunT = Buildings.Utilities.Math.Functions.biquadratic(a=per.EIRFunT, x1=TEvaLvg_degC, x2=TConEnt_degC);
        // Chiller energy input ratio quadratic curve
        EIRFunPLR   = per.EIRFunPLR[1]+per.EIRFunPLR[2]*PLR2+per.EIRFunPLR[3]*PLR2^2;
      else
        capFunT   = 0;
        EIRFunT   = 0;
        EIRFunPLR = 0;
      end if;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                       graphics={
            Rectangle(
              extent={{-99,-54},{102,-66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,-66},{0,-54}},
              lineColor={0,0,127},
              pattern=LinePattern.None,
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-104,66},{98,54}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-2,54},{98,66}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-44,52},{-40,12}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-56,70},{58,52}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-42,2},{-52,12},{-32,12},{-42,2}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-44,-10},{-40,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{38,52},{42,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-56,-50},{58,-68}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{18,24},{62,-18}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{40,24},{22,-8},{58,-8},{40,24}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
    defaultComponentName="chi",
    Documentation(info="<html>
<p>
Model of an electric chiller, based on the DOE-2.1 chiller model and
the EnergyPlus chiller model <code>Chiller:Electric:EIR</code>.
</p>
<p> This model uses three functions to predict capacity and power consumption:
</p>
<ul>
<li>
A biquadratic function is used to predict cooling capacity as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
<li>
A quadratic functions is used to predict power input to cooling capacity ratio with respect to the part load ratio.
</li>
<li>
A biquadratic functions is used to predict power input to cooling capacity ratio as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
</ul>
<p>
These curves are stored in the data record <code>per</code> and are available from
<a href=\"Buildings.Fluid.Chillers.Data.ElectricEIR\">
Buildings.Fluid.Chillers.Data.ElectricEIR</a>.
Additional performance curves can be developed using
two available techniques (Hydeman and Gillespie, 2002). The first technique is called the
Least-squares Linear Regression method and is used when sufficient performance data exist
to employ standard least-square linear regression techniques. The second technique is called
Reference Curve Method and is used when insufficient performance data exist to apply linear
regression techniques. A detailed description of both techniques can be found in
Hydeman and Gillespie (2002).
</p>
<p>
The model takes as an input the set point for the leaving chilled water temperature,
which is met if the chiller has sufficient capacity.
Thus, the model has a built-in, ideal temperature control.
The model has three tests on the part load ratio and the cycling ratio:
</p>
<ol>
<li>
The test<pre>
  PLR1 =min(QEva_flow_set/QEva_flow_ava, per.PLRMax);
</pre>
ensures that the chiller capacity does not exceed the chiller capacity specified
by the parameter <code>per.PLRMax</code>.
</li>
<li>
The test <pre>
  CR = min(PLR1/per.PRLMin, 1.0);
</pre>
computes a cycling ratio. This ratio expresses the fraction of time
that a chiller would run if it were to cycle because its load is smaller than the
minimal load at which it can operate.
Note that this model continuously operates even if the part load ratio is below the minimum part load ratio.
Its leaving evaporator and condenser temperature can therefore be considered as an
average temperature between the modes where the compressor is off and on.
</li>
<li>
The test <pre>
  PLR2 = max(per.PLRMinUnl, PLR1);
</pre>
computes the part load ratio of the compressor.
The assumption is that for a part load ratio below <code>per.PLRMinUnl</code>,
the chiller uses hot gas bypass to reduce the capacity, while the compressor
power draw does not change.
</li>
</ol>
<p>
The electric power only contains the power for the compressor, but not any power for pumps or fans.
</p>
<p>
The model can be parametrized to compute a transient
or steady-state response.
The transient response of the boiler is computed using a first
order differential equation for the evaporator and condenser fluid volumes.
The chiller outlet temperatures are equal to the temperatures of these lumped volumes.
</p>
<h4>References</h4>
<ul>
<li>
Hydeman, M. and K.L. Gillespie. 2002. Tools and Techniques to Calibrate Electric Chiller
Component Models. <i>ASHRAE Transactions</i>, AC-02-9-1.
</li>
</ul>
</html>",
    revisions="<html>
<ul>
<li>
March 12, 2015, by Michael Wetter:<br/>
Refactored model to make it once continuously differentiable.
This is for issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/373\">373</a>.
</li>
<li>
Jan. 9, 2011, by Michael Wetter:<br/>
Added input signal to switch chiller off.
</li>
<li>
Sep. 8, 2010, by Michael Wetter:<br/>
Revised model and included it in the Buildings library.
</li>
<li>
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"));
    end ElectricEIRCW;

    package BaseClassesChiller
      "Package with base classes for Buildings.Fluid.Chillers"
      extends Modelica.Icons.BasesPackage;

      partial model Carnot
        extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
          m1_flow_nominal = QCon_flow_nominal/cp1_default/dTCon_nominal,
          m2_flow_nominal = QEva_flow_nominal/cp2_default/dTEva_nominal);

        parameter Modelica.SIunits.HeatFlowRate QEva_flow_nominal(max=0)
          "Nominal cooling heat flow rate (QEva_flow_nominal < 0)"
          annotation (Dialog(group="Nominal condition"));
        parameter Modelica.SIunits.HeatFlowRate QCon_flow_nominal(min=0)
          "Nominal heating flow rate"
          annotation (Dialog(group="Nominal condition"));

        parameter Modelica.SIunits.TemperatureDifference dTEva_nominal(
          final max=0) = -10 "Temperature difference evaporator outlet-inlet"
          annotation (Dialog(group="Nominal condition"));
        parameter Modelica.SIunits.TemperatureDifference dTCon_nominal(
          final min=0) = 10 "Temperature difference condenser outlet-inlet"
          annotation (Dialog(group="Nominal condition"));

        // Efficiency
        parameter Boolean use_eta_Carnot_nominal = true
          "Set to true to use Carnot effectiveness etaCarnot_nominal rather than COP_nominal"
          annotation(Dialog(group="Efficiency"));
        parameter Real etaCarnot_nominal(unit="1") = COP_nominal/
          (TUseAct_nominal/(TCon_nominal+TAppCon_nominal - (TEva_nominal-TAppEva_nominal)))
          "Carnot effectiveness (=COP/COP_Carnot) used if use_eta_Carnot_nominal = true"
          annotation (Dialog(group="Efficiency", enable=use_eta_Carnot_nominal));

        parameter Real COP_nominal(unit="1") = etaCarnot_nominal*TUseAct_nominal/
          (TCon_nominal+TAppCon_nominal - (TEva_nominal-TAppEva_nominal))
          "Coefficient of performance at TEva_nominal and TCon_nominal, used if use_eta_Carnot_nominal = false"
          annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

        parameter Modelica.SIunits.Temperature TCon_nominal = 303.15
          "Condenser temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
          annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));
        parameter Modelica.SIunits.Temperature TEva_nominal = 278.15
          "Evaporator temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
          annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

        parameter Real a[:] = {1}
          "Coefficients for efficiency curve (need p(a=a, yPL=1)=1)"
          annotation (Dialog(group="Efficiency"));

        parameter Modelica.SIunits.Pressure dp1_nominal(displayUnit="Pa")
          "Pressure difference over condenser"
          annotation (Dialog(group="Nominal condition"));
        parameter Modelica.SIunits.Pressure dp2_nominal(displayUnit="Pa")
          "Pressure difference over evaporator"
          annotation (Dialog(group="Nominal condition"));

        parameter Modelica.SIunits.TemperatureDifference TAppCon_nominal(min=0) = if cp1_default < 1500 then 5 else 2
          "Temperature difference between refrigerant and working fluid outlet in condenser"
          annotation (Dialog(group="Efficiency"));

        parameter Modelica.SIunits.TemperatureDifference TAppEva_nominal(min=0) = if cp2_default < 1500 then 5 else 2
          "Temperature difference between refrigerant and working fluid outlet in evaporator"
          annotation (Dialog(group="Efficiency"));

        parameter Boolean homotopyInitialization=true "= true, use homotopy method"
          annotation (Dialog(tab="Advanced"));

        parameter Boolean from_dp1=false
          "= true, use m_flow = f(dp) else dp = f(m_flow)"
          annotation (Dialog(tab="Flow resistance", group="Condenser"));
        parameter Boolean from_dp2=false
          "= true, use m_flow = f(dp) else dp = f(m_flow)"
          annotation (Dialog(tab="Flow resistance", group="Evaporator"));

        parameter Boolean linearizeFlowResistance1=false
          "= true, use linear relation between m_flow and dp for any flow rate"
          annotation (Dialog(tab="Flow resistance", group="Condenser"));
        parameter Boolean linearizeFlowResistance2=false
          "= true, use linear relation between m_flow and dp for any flow rate"
          annotation (Dialog(tab="Flow resistance", group="Evaporator"));

        parameter Real deltaM1(final unit="1")=0.1
          "Fraction of nominal flow rate where flow transitions to laminar"
          annotation (Dialog(tab="Flow resistance", group="Condenser"));
        parameter Real deltaM2(final unit="1")=0.1
          "Fraction of nominal flow rate where flow transitions to laminar"
          annotation (Dialog(tab="Flow resistance", group="Evaporator"));

        parameter Modelica.SIunits.Time tau1=60
          "Time constant at nominal flow rate (used if energyDynamics1 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
          annotation (Dialog(tab="Dynamics", group="Condenser"));
        parameter Modelica.SIunits.Time tau2=60
          "Time constant at nominal flow rate (used if energyDynamics2 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
          annotation (Dialog(tab="Dynamics", group="Evaporator"));

        parameter Modelica.SIunits.Temperature T1_start=Medium1.T_default
          "Initial or guess value of set point"
          annotation (Dialog(tab="Dynamics", group="Condenser"));
        parameter Modelica.SIunits.Temperature T2_start=Medium2.T_default
          "Initial or guess value of set point"
          annotation (Dialog(tab="Dynamics", group="Evaporator"));

        parameter Modelica.Fluid.Types.Dynamics energyDynamics=
          Modelica.Fluid.Types.Dynamics.SteadyState "Type of energy balance: dynamic (3 initialization options) or steady state"
          annotation (Dialog(tab="Dynamics", group="Evaporator and condenser"));

        Modelica.Blocks.Interfaces.RealOutput QCon_flow(
          final quantity="HeatFlowRate",
          final unit="W") "Actual heating heat flow rate added to fluid 1"
          annotation (Placement(transformation(extent={{100,80},{120,100}}),
              iconTransformation(extent={{100,80},{120,100}})));

        Modelica.Blocks.Interfaces.RealOutput P(
          final quantity="Power",
          final unit="W") "Electric power consumed by compressor"
          annotation (Placement(transformation(extent={{100,-10},{120,10}}),
              iconTransformation(extent={{100,-10},{120,10}})));

        Modelica.Blocks.Interfaces.RealOutput QEva_flow(
          final quantity="HeatFlowRate",
          final unit="W") "Actual cooling heat flow rate removed from fluid 2"
          annotation (Placement(transformation(extent={{100,-100},{120,-80}}),
              iconTransformation(extent={{100,-100},{120,-80}})));

        Real yPL(final unit="1", min=0) = if COP_is_for_cooling
           then QEva_flow/QEva_flow_nominal
           else QCon_flow/QCon_flow_nominal "Part load ratio";

        Real etaPL(final unit = "1")=
          if evaluate_etaPL
            then 1
          else Buildings.Utilities.Math.Functions.polynomial(a=a, x=yPL)
          "Efficiency due to part load (etaPL(yPL=1)=1)";

        Real COP(min=0, final unit="1") = etaCarnot_nominal_internal * COPCar * etaPL
          "Coefficient of performance";

        Real COPCar(min=0) = TUseAct/Buildings.Utilities.Math.Functions.smoothMax(
          x1=1,
          x2=TConAct - TEvaAct,
          deltaX=0.25) "Carnot efficiency";

        Modelica.SIunits.Temperature TConAct(start=TCon_nominal + TAppCon_nominal)=
          Medium1.temperature(staB1) + QCon_flow/QCon_flow_nominal*TAppCon_nominal
          "Condenser temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";

        Modelica.SIunits.Temperature TEvaAct(start=TEva_nominal - TAppEva_nominal)=
          Medium2.temperature(staB2) - QEva_flow/QEva_flow_nominal*TAppEva_nominal
          "Evaporator temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";

      protected
        constant Boolean COP_is_for_cooling
          "Set to true if the specified COP is for cooling";

        parameter Real etaCarnot_nominal_internal(unit="1")=
          if use_eta_Carnot_nominal
            then etaCarnot_nominal
            else COP_nominal/
                 (TUseAct_nominal / (TCon_nominal + TAppCon_nominal - (TEva_nominal - TAppEva_nominal)))
          "Carnot effectiveness (=COP/COP_Carnot) used to compute COP";

        // For Carnot_y, computing etaPL = f(yPL) introduces a nonlinear equation.
        // The parameter below avoids this if a = {1}.
        final parameter Boolean evaluate_etaPL=
          (size(a, 1) == 1 and abs(a[1] - 1)  < Modelica.Constants.eps)
          "Flag, true if etaPL should be computed as it depends on yPL"
          annotation(Evaluate=true);

        final parameter Modelica.SIunits.Temperature TUseAct_nominal=
          if COP_is_for_cooling
            then TEva_nominal - TAppEva_nominal
            else TCon_nominal + TAppCon_nominal
          "Nominal evaporator temperature for chiller or condenser temperature for heat pump, taking into account pinch temperature between fluid and refrigerant";
        Modelica.SIunits.Temperature TUseAct=if COP_is_for_cooling then TEvaAct else TConAct
          "Temperature of useful heat (evaporator for chiller, condenser for heat pump), taking into account pinch temperature between fluid and refrigerant";

        final parameter Modelica.SIunits.SpecificHeatCapacity cp1_default=
          Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
            p = Medium1.p_default,
            T = Medium1.T_default,
            X = Medium1.X_default))
          "Specific heat capacity of medium 1 at default medium state";

        final parameter Modelica.SIunits.SpecificHeatCapacity cp2_default=
          Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
            p = Medium2.p_default,
            T = Medium2.T_default,
            X = Medium2.X_default))
          "Specific heat capacity of medium 2 at default medium state";

        Medium1.ThermodynamicState staA1 = Medium1.setState_phX(
          port_a1.p,
          inStream(port_a1.h_outflow),
          inStream(port_a1.Xi_outflow)) "Medium properties in port_a1";
        Medium1.ThermodynamicState staB1 = Medium1.setState_phX(
          port_b1.p,
          port_b1.h_outflow,
          port_b1.Xi_outflow) "Medium properties in port_b1";
        Medium2.ThermodynamicState staA2 = Medium2.setState_phX(
          port_a2.p,
          inStream(port_a2.h_outflow),
          inStream(port_a2.Xi_outflow)) "Medium properties in port_a2";
        Medium2.ThermodynamicState staB2 = Medium2.setState_phX(
          port_b2.p,
          port_b2.h_outflow,
          port_b2.Xi_outflow) "Medium properties in port_b2";

        replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface con
          constrainedby Buildings.Fluid.Interfaces.PartialTwoPortInterface(
          redeclare final package Medium = Medium1,
          final allowFlowReversal=allowFlowReversal1,
          final m_flow_nominal=m1_flow_nominal,
          final m_flow_small=m1_flow_small,
          final show_T=false) "Condenser"
          annotation (Placement(transformation(extent={{-10,50},{10,70}})));

        replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface eva
          constrainedby Buildings.Fluid.Interfaces.PartialTwoPortInterface(
          redeclare final package Medium = Medium2,
          final allowFlowReversal=allowFlowReversal2,
          final m_flow_nominal=m2_flow_nominal,
          final m_flow_small=m2_flow_small,
          final show_T=false) "Evaporator"
          annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));

      initial equation
        assert(dTEva_nominal < 0,
          "Parameter dTEva_nominal must be negative.");
        assert(dTCon_nominal > 0,
          "Parameter dTCon_nominal must be positive.");

        assert(abs(Buildings.Utilities.Math.Functions.polynomial(
               a=a, x=1)-1) < 0.01, "Efficiency curve is wrong. Need etaPL(y=1)=1.");
        assert(etaCarnot_nominal_internal < 1,   "Parameters lead to etaCarnot_nominal > 1. Check parameters.");

      equation
        connect(port_a2, eva.port_a)
          annotation (Line(points={{100,-60},{56,-60},{10,-60}}, color={0,127,255}));
        connect(eva.port_b, port_b2) annotation (Line(points={{-10,-60},{-100,-60}},
                       color={0,127,255}));
        connect(port_a1, con.port_a)
          annotation (Line(points={{-100,60},{-56,60},{-10,60}}, color={0,127,255}));
        connect(con.port_b, port_b1)
          annotation (Line(points={{10,60},{56,60},{100,60}}, color={0,127,255}));

        annotation (
        Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},
                  {100,100}}),       graphics={
              Rectangle(
                extent={{-70,80},{70,-80}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,68},{58,50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,-52},{58,-70}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-103,64},{98,54}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-2,54},{98,64}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-101,-56},{100,-66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,-66},{0,-56}},
                lineColor={0,0,127},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,0},{-52,-12},{-32,-12},{-42,0}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,0},{-52,10},{-32,10},{-42,0}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,50},{-40,10}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,-12},{-40,-52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,50},{42,-52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{18,22},{62,-20}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{40,22},{22,-10},{58,-10},{40,22}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{0,68},{0,90},{90,90},{100,90}},
                                                       color={0,0,255}),
              Line(points={{0,-70},{0,-90},{100,-90}}, color={0,0,255}),
              Line(points={{62,0},{100,0}},                 color={0,0,255})}),
            Documentation(info="<html>
<p>
This is the base class for the Carnot chiller and the Carnot heat pump
whose coefficient of performance COP changes
with temperatures in the same way as the Carnot efficiency changes.
</p>
<p>
The model allows to either specify the Carnot effectivness
<i>&eta;<sub>Carnot,0</sub></i>, or
a <i>COP<sub>0</sub></i>
at the nominal conditions, together with
the evaporator temperature <i>T<sub>eva,0</sub></i> and
the condenser temperature <i>T<sub>con,0</sub></i>, in which
case the model computes the Carnot effectivness as
</p>
<p align=\"center\" style=\"font-style:italic;\">
&eta;<sub>Carnot,0</sub> =
  COP<sub>0</sub>
&frasl;  (T<sub>use,0</sub> &frasl; (T<sub>con,0</sub>-T<sub>eva,0</sub>)),
</p>
<p>
where
<i>T<sub>use</sub></i> is the temperature of the the useful heat,
e.g., the evaporator temperature for a chiller or the condenser temperature
for a heat pump.
</p>
<p>
The COP is computed as the product
</p>
<p align=\"center\" style=\"font-style:italic;\">
  COP = &eta;<sub>Carnot,0</sub> COP<sub>Carnot</sub> &eta;<sub>PL</sub>,
</p>
<p>
where <i>COP<sub>Carnot</sub></i> is the Carnot efficiency and
<i>&eta;<sub>PL</sub></i> is the part load efficiency, expressed using
a polynomial.
This polynomial has the form
</p>
<p align=\"center\" style=\"font-style:italic;\">
  &eta;<sub>PL</sub> = a<sub>1</sub> + a<sub>2</sub> y + a<sub>3</sub> y<sup>2</sup> + ...
</p>
<p>
where <i>y &isin; [0, 1]</i> is
either the part load for cooling in case of a chiller, or the part load of heating in
case of a heat pump, and the coefficients <i>a<sub>i</sub></i>
are declared by the parameter <code>a</code>.
</p>
<h4>Implementation</h4>
<p>
To make this base class applicable to chiller or heat pumps, it uses
the boolean constant <code>COP_is_for_cooling</code>.
Depending on its value, the equations for the coefficient of performance
and the part load ratio are set up.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 16, 2017, by Michael Wetter:<br/>
Added temperature difference between fluids in condenser and evaporator
for computation of nominal COP and effectiveness.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
March 28, 2017, by Felix Buenning:<br/>
Added temperature difference between fluids in condenser and evaporator.
The difference is based on discussions with Emerson Climate Technologies.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
January 2, 2017, by Filip Jorissen:<br/>
Removed option for choosing what temperature
should be used to compute the Carnot efficiency.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/497\">
issue 497</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
First implementation of this base class.
</li>
</ul>
</html>"));
      end Carnot;

      partial model PartialCarnot_T
        "Partial model for chiller with performance curve adjusted based on Carnot efficiency"
        extends Task2_3.Task2c.BaseClassesChiller.Carnot;

      protected
        Modelica.Blocks.Sources.RealExpression PEle "Electrical power consumption"
          annotation (Placement(transformation(extent={{40,-10},{60,10}})));

      equation
        connect(PEle.y, P)
          annotation (Line(points={{61,0},{110,0},{110,0}}, color={0,0,127}));
        annotation (
      Documentation(info="<html>
<p>
This is a partial model of a chiller whose coefficient of performance (COP) changes
with temperatures in the same way as the Carnot efficiency changes.
This base class is used for the Carnot chiller and Carnot heat pump
that uses the compressor part load ratio as the control signal.
</p>
</html>",
      revisions="<html>
<ul>
<li>
January 26, 2016, by Michael Wetter:<br/>
First implementation of this base class.
</li>
</ul>
</html>"));
      end PartialCarnot_T;

      partial model PartialCarnot_y
        "Partial chiller model with performance curve adjusted based on Carnot efficiency"
        extends Carnot(
          final QCon_flow_nominal= P_nominal - QEva_flow_nominal,
          final QEva_flow_nominal = if COP_is_for_cooling
                                    then -P_nominal * COP_nominal
                                    else -P_nominal * (COP_nominal-1),
          redeclare Buildings.Fluid.HeatExchangers.HeaterCooler_u con(
            final from_dp=from_dp1,
            final dp_nominal=dp1_nominal,
            final linearizeFlowResistance=linearizeFlowResistance1,
            final deltaM=deltaM1,
            final tau=tau1,
            final T_start=T1_start,
            final energyDynamics=energyDynamics,
            final massDynamics=energyDynamics,
            final homotopyInitialization=homotopyInitialization,
            final Q_flow_nominal=QCon_flow_nominal),
            redeclare Buildings.Fluid.HeatExchangers.HeaterCooler_u eva(
            final from_dp=from_dp2,
            final dp_nominal=dp2_nominal,
            final linearizeFlowResistance=linearizeFlowResistance2,
            final deltaM=deltaM2,
            final tau=tau2,
            final T_start=T2_start,
            final energyDynamics=energyDynamics,
            final massDynamics=energyDynamics,
            final homotopyInitialization=homotopyInitialization,
            final Q_flow_nominal=QEva_flow_nominal));

        parameter Modelica.SIunits.Power P_nominal(min=0)
          "Nominal compressor power (at y=1)"
          annotation (Dialog(group="Nominal condition"));

        Modelica.Blocks.Interfaces.RealInput y(min=0, max=1, unit="1")
          "Part load ratio of compressor"
          annotation (Placement(transformation(extent={{-140,70},{-100,110}})));

      protected
        Modelica.SIunits.HeatFlowRate QCon_flow_internal(start=QCon_flow_nominal)=
          P - QEva_flow_internal "Condenser heat input";
        Modelica.SIunits.HeatFlowRate QEva_flow_internal(start=QEva_flow_nominal)=
          if COP_is_for_cooling then -COP * P else (1-COP)*P "Evaporator heat input";

        Modelica.Blocks.Sources.RealExpression yEva_flow_in(
          y=QEva_flow_internal/QEva_flow_nominal)
          "Normalized evaporator heat flow rate"
          annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
        Modelica.Blocks.Sources.RealExpression yCon_flow_in(
          y=QCon_flow_internal/QCon_flow_nominal)
          "Normalized condenser heat flow rate"
          annotation (Placement(transformation(extent={{-80,30},{-60,50}})));

        Modelica.Blocks.Math.Gain PEle(final k=P_nominal)
          "Electrical power consumption"
          annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      equation

        connect(PEle.y, P)
          annotation (Line(points={{81,0},{110,0}}, color={0,0,127}));
        connect(PEle.u, y) annotation (Line(points={{58,0},{58,0},{40,0},{40,90},{-92,
                90},{-120,90}},          color={0,0,127}));
        connect(yEva_flow_in.y, eva.u) annotation (Line(points={{-59,-40},{20,-40},{20,
                -54},{12,-54}}, color={0,0,127}));
        connect(yCon_flow_in.y, con.u) annotation (Line(points={{-59,40},{-48,40},{-40,
                40},{-40,66},{-12,66}}, color={0,0,127}));
        connect(con.Q_flow, QCon_flow) annotation (Line(points={{11,66},{20,66},{80,66},
                {80,90},{110,90}}, color={0,0,127}));
        connect(eva.Q_flow, QEva_flow) annotation (Line(points={{-11,-54},{-20,-54},{-20,
                -90},{110,-90}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={
              Rectangle(
                extent={{-56,68},{58,50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,-52},{58,-70}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-103,64},{98,54}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-2,54},{98,64}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-101,-56},{100,-66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,-66},{0,-56}},
                lineColor={0,0,127},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,0},{-52,-12},{-32,-12},{-42,0}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,0},{-52,10},{-32,10},{-42,0}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,50},{-40,10}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,-12},{-40,-52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,50},{42,-52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{18,22},{62,-20}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{40,22},{22,-10},{58,-10},{40,22}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-130,128},{-78,106}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="y"),
              Text(extent={{66,28},{116,14}},   textString="P",
                lineColor={0,0,127}),
              Line(points={{-100,90},{-80,90},{-80,14},{22,14}},
                                                          color={0,0,255}),
              Line(points={{62,0},{100,0}},                 color={0,0,255})}),
      defaultComponentName="chi",
      Documentation(info="<html>
<p>
This is a partial model of a chiller whose coefficient of performance (COP) changes
with temperatures in the same way as the Carnot efficiency changes.
This base class is used for the Carnot chiller and Carnot heat pump
that uses the leaving fluid temperature as the control signal.
</p>
</html>",
      revisions="<html>
<ul>
<li>
June 15, 2017, by Michael Wetter:<br/>
Added <code>min</code> attribute to parameter <code>P_nominal</code>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Implemented in the Annex 60 library the models
<a href=\"modelica://Buildings.Fluid.Chillers.Carnot_y\">Buildings.Fluid.Chillers.Carnot_y</a>
and
<a href=\"modelica://Buildings.Fluid.HeatPumps.Carnot_y\">Buildings.Fluid.HeatPumps.Carnot_y</a>
and refactored these models to use the same base class.<br/>
Implemented the removal of the flow direction dependency of
<code>staA1</code>, <code>staB1</code>, <code>staA2</code> and <code>staB2</code> as the
efficiency of the Carnot machine should only be computed in the design flow direction,
as corrected by Damien Picard.
</li>
<li>
December 18, 2015, by Michael Wetter:<br/>
Corrected wrong computation of <code>staB1</code> and <code>staB2</code>
which mistakenly used the <code>inStream</code> operator
for the configuration without flow reversal.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/476\">
issue 476</a>.
</li>
<li>
November 25, 2015 by Michael Wetter:<br/>
Changed sign convention for <code>dTEva_nominal</code> to be consistent with
other models.
The model will still work with the old values for <code>dTEva_nominal</code>,
but it will write a warning so that users can transition their models.
<br/>
Corrected <code>assert</code> statement for the efficiency curve.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/468\">
issue 468</a>.
</li>
<li>
September 3, 2015 by Michael Wetter:<br/>
Expanded documentation.
</li>
<li>
May 6, 2015 by Michael Wetter:<br/>
Added <code>prescribedHeatFlowRate=true</code> for <code>vol2</code>.
</li>
<li>
October 9, 2013 by Michael Wetter:<br/>
Reimplemented the computation of the port states to avoid using
the conditionally removed variables <code>sta_a1</code>,
<code>sta_a2</code>, <code>sta_b1</code> and <code>sta_b2</code>.
</li>
<li>
May 10, 2013 by Michael Wetter:<br/>
Added electric power <code>P</code> as an output signal.
</li>
<li>
October 11, 2010 by Michael Wetter:<br/>
Fixed bug in energy balance.
</li>
<li>
March 3, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
      end PartialCarnot_y;

      partial model PartialElectric
        "Partial model for electric chiller based on the model in DOE-2, CoolTools and EnergyPlus"
        extends Task2_3.Task2c.BaseClassesChiller.FourPortHeatMassExchanger2(
          m1_flow_nominal=mCon_flow_nominal,
          m2_flow_nominal=mEva_flow_nominal,
          T1_start=273.15 + 25,
          T2_start=273.15 + 5,
          redeclare final Buildings.Fluid.MixingVolumes.MixingVolume vol2(
            V=m2_flow_nominal*tau2/rho2_nominal,
            nPorts=2,
            final prescribedHeatFlowRate=true),
          vol1(final prescribedHeatFlowRate=true));

        Modelica.Blocks.Interfaces.BooleanInput on
          "Set to true to enable compressor, or false to disable compressor"
          annotation (Placement(transformation(extent={{-140,10},{-100,50}}),
              iconTransformation(extent={{-140,10},{-100,50}})));
        Modelica.Blocks.Interfaces.RealInput TSet(unit="K", displayUnit="degC")
          "Set point for leaving chilled water temperature"
          annotation (Placement(transformation(extent={{-140,-50},{-100,-10}}),
              iconTransformation(extent={{-140,-50},{-100,-10}})));

        Modelica.SIunits.Temperature TEvaEnt "Evaporator entering temperature";
        Modelica.SIunits.Temperature TEvaLvg "Evaporator leaving temperature";
        Modelica.SIunits.Temperature TConEnt "Condenser entering temperature";
        Modelica.SIunits.Temperature TConLvg "Condenser leaving temperature";

        Modelica.SIunits.Efficiency COP "Coefficient of performance";
        Modelica.SIunits.HeatFlowRate QCon_flow "Condenser heat input";
        Modelica.SIunits.HeatFlowRate QEva_flow "Evaporator heat input";
        Modelica.Blocks.Interfaces.RealOutput P(final quantity="Power", unit="W")
          "Electric power consumed by compressor"
          annotation (Placement(transformation(extent={{100,80},{120,100}}),
              iconTransformation(extent={{100,80},{120,100}})));

        Real capFunT(min=0, nominal=1, start=1, unit="1")
          "Cooling capacity factor function of temperature curve";
        Modelica.SIunits.Efficiency EIRFunT(nominal=1, start=1)
          "Power input to cooling capacity ratio function of temperature curve";
        Modelica.SIunits.Efficiency EIRFunPLR(nominal=1, start=1)
          "Power input to cooling capacity ratio function of part load ratio";
        Real PLR1(min=0, nominal=1, start=1, unit="1") "Part load ratio";
        Real PLR2(min=0, nominal=1, start=1, unit="1") "Part load ratio";
        Real CR(min=0, nominal=1,  start=1, unit="1") "Cycling ratio";

      protected
        Modelica.SIunits.HeatFlowRate QEva_flow_ava(nominal=QEva_flow_nominal,start=QEva_flow_nominal)
          "Cooling capacity available at evaporator";
        Modelica.SIunits.HeatFlowRate QEva_flow_set(nominal=QEva_flow_nominal,start=QEva_flow_nominal)
          "Cooling capacity required to cool to set point temperature";
        Modelica.SIunits.SpecificEnthalpy hSet
          "Enthalpy setpoint for leaving chilled water";
        // Performance data
        parameter Modelica.SIunits.HeatFlowRate QEva_flow_nominal(max=0)
          "Reference capacity (negative number)";
        parameter Modelica.SIunits.Efficiency COP_nominal
          "Reference coefficient of performance";
        parameter Real PLRMax(min=0, unit="1") "Maximum part load ratio";
        parameter Real PLRMinUnl(min=0, unit="1") "Minimum part unload ratio";
        parameter Real PLRMin(min=0, unit="1") "Minimum part load ratio";
        parameter Modelica.SIunits.Efficiency etaMotor(max=1)
          "Fraction of compressor motor heat entering refrigerant";
        parameter Modelica.SIunits.MassFlowRate mEva_flow_nominal
          "Nominal mass flow at evaporator";
        parameter Modelica.SIunits.MassFlowRate mCon_flow_nominal
          "Nominal mass flow at condenser";
        parameter Modelica.SIunits.Temperature TEvaLvg_nominal
          "Temperature of fluid leaving evaporator at nominal condition";
        final parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC
          TEvaLvg_nominal_degC=
          Modelica.SIunits.Conversions.to_degC(TEvaLvg_nominal)
          "Temperature of fluid leaving evaporator at nominal condition";
        Modelica.SIunits.Conversions.NonSIunits.Temperature_degC TEvaLvg_degC
          "Temperature of fluid leaving evaporator";
        parameter Modelica.SIunits.HeatFlowRate Q_flow_small = QEva_flow_nominal*1E-9
          "Small value for heat flow rate or power, used to avoid division by zero";
        Buildings.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloEva
          "Prescribed heat flow rate"
          annotation (Placement(transformation(extent={{-39,-50},{-19,-30}})));
        Buildings.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloCon
          "Prescribed heat flow rate"
          annotation (Placement(transformation(extent={{-37,30},{-17,50}})));
        Modelica.Blocks.Sources.RealExpression QEva_flow_in(y=QEva_flow)
          "Evaporator heat flow rate"
          annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
        Modelica.Blocks.Sources.RealExpression QCon_flow_in(y=QCon_flow)
          "Condenser heat flow rate"
          annotation (Placement(transformation(extent={{-80,30},{-60,50}})));

      initial equation
        assert(QEva_flow_nominal < 0, "Parameter QEva_flow_nominal must be smaller than zero.");
        assert(Q_flow_small < 0, "Parameter Q_flow_small must be smaller than zero.");
        assert(PLRMinUnl >= PLRMin, "Parameter PLRMinUnl must be bigger or equal to PLRMin");
        assert(PLRMax > PLRMinUnl, "Parameter PLRMax must be bigger than PLRMinUnl");
      equation
        // Condenser temperatures
        TConEnt = Medium1.temperature(Medium1.setState_phX(port_a1.p, inStream(port_a1.h_outflow)));
        TConLvg = vol1.heatPort.T;
        // Evaporator temperatures
        TEvaEnt = Medium2.temperature(Medium2.setState_phX(port_a2.p, inStream(port_a2.h_outflow)));
        TEvaLvg = vol2.heatPort.T;
        TEvaLvg_degC=Modelica.SIunits.Conversions.to_degC(TEvaLvg);

        // Enthalpy of temperature setpoint
        hSet = Medium2.specificEnthalpy_pTX(
                 p=port_b2.p,
                 T=TSet,
                 X=cat(1, port_b2.Xi_outflow, {1-sum(port_b2.Xi_outflow)}));

        if on then
          // Available cooling capacity
          QEva_flow_ava = QEva_flow_nominal*capFunT;
          // Cooling capacity required to chill water to setpoint
          QEva_flow_set = Buildings.Utilities.Math.Functions.smoothMin(
            x1 = m2_flow*(hSet-inStream(port_a2.h_outflow)),
            x2= Q_flow_small,
            deltaX=-Q_flow_small/100);

          // Part load ratio
          PLR1 = Buildings.Utilities.Math.Functions.smoothMin(
            x1 = QEva_flow_set/(QEva_flow_ava+Q_flow_small),
            x2 = PLRMax,
            deltaX=PLRMax/100);
          // PLR2 is the compressor part load ratio. The lower bound PLRMinUnl is
          // since for PLR1<PLRMinUnl, the chiller uses hot gas bypass, under which
          // condition the compressor power is assumed to be the same as if the chiller
          // were to operate at PLRMinUnl
          PLR2 = Buildings.Utilities.Math.Functions.smoothMax(
            x1 = PLRMinUnl,
            x2 = PLR1,
            deltaX = PLRMinUnl/100);

          // Cycling ratio.
          // Due to smoothing, this can be about deltaX/10 above 1.0
          CR = Buildings.Utilities.Math.Functions.smoothMin(
            x1 = PLR1/PLRMin,
            x2 = 1,
            deltaX=0.001);

          // Compressor power.
          P = -QEva_flow_ava/COP_nominal*EIRFunT*EIRFunPLR*CR;
          // Heat flow rates into evaporator and condenser
          // Q_flow_small is a negative number.
          QEva_flow = Buildings.Utilities.Math.Functions.smoothMax(
            x1 = QEva_flow_set,
            x2 = QEva_flow_ava,
            deltaX= -Q_flow_small/10);

        //QEva_flow = max(QEva_flow_set, QEva_flow_ava);
          QCon_flow = -QEva_flow + P*etaMotor;
          // Coefficient of performance
          COP = -QEva_flow/(P-Q_flow_small);
        else
          QEva_flow_ava = 0;
          QEva_flow_set = 0;
          PLR1 = 0;
          PLR2 = 0;
          CR   = 0;
          P    = 0;
          QEva_flow = 0;
          QCon_flow = 0;
          COP  = 0;
        end if;

        connect(QCon_flow_in.y, preHeaFloCon.Q_flow) annotation (Line(
            points={{-59,40},{-37,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(preHeaFloCon.port, vol1.heatPort) annotation (Line(
            points={{-17,40},{-10,40},{-10,60}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(QEva_flow_in.y, preHeaFloEva.Q_flow) annotation (Line(
            points={{-59,-40},{-39,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(preHeaFloEva.port, vol2.heatPort) annotation (Line(
            points={{-19,-40},{12,-40},{12,-60}},
            color={191,0,0},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}),
                         graphics={
              Text(extent={{62,96},{112,82}},   textString="P",
                lineColor={0,0,127}),
              Text(extent={{-94,-24},{-48,-36}},  textString="T_CHWS",
                lineColor={0,0,127}),
              Rectangle(
                extent={{-99,-54},{102,-66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,-66},{0,-54}},
                lineColor={0,0,127},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-104,66},{98,54}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-2,54},{98,66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,52},{-40,12}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,70},{58,52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,2},{-52,12},{-32,12},{-42,2}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,-10},{-40,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,52},{42,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,-50},{58,-68}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{18,24},{62,-18}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{40,24},{22,-8},{58,-8},{40,24}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(extent={{-108,36},{-62,24}},
                lineColor={0,0,127},
                textString="on")}),
      Documentation(info="<html>
<p>
Base class for model of an electric chiller, based on the DOE-2.1 chiller model and the
CoolTools chiller model that are implemented in EnergyPlus as the models
<code>Chiller:Electric:EIR</code> and <code>Chiller:Electric:ReformulatedEIR</code>.
</p>
<p>
The model takes as an input the set point for the leaving chilled water temperature,
which is met if the chiller has sufficient capacity.
Thus, the model has a built-in, ideal temperature control.
The model has three tests on the part load ratio and the cycling ratio:
</p>
<ol>
<li>
The test
<pre>
  PLR1 =min(QEva_flow_set/QEva_flow_ava, PLRMax);
</pre>
ensures that the chiller capacity does not exceed the chiller capacity specified
by the parameter <code>PLRMax</code>.
</li>
<li>
The test <pre>
  CR = min(PLR1/per.PRLMin, 1.0);
</pre>
computes a cycling ratio. This ratio expresses the fraction of time
that a chiller would run if it were to cycle because its load is smaller than
the minimal load at which it can operature. Notice that this model does continuously operature even if
the part load ratio is below the minimum part load ratio. Its leaving evaporator and condenser temperature
can therefore be considered as an
average temperature between the modes where the compressor is off and on.
</li>
<li>
The test <pre>
  PLR2 = max(PLRMinUnl, PLR1);
</pre>
computes the part load ratio of the compressor.
The assumption is that for a part load ratio below <code>PLRMinUnl</code>,
the chiller uses hot gas bypass to reduce the capacity, while the compressor
power draw does not change.
</li>
</ol>
<p>
The electric power only contains the power for the compressor, but not any power for pumps or fans.
</p>
<h4>Implementation</h4>
<p>
Models that extend from this base class need to provide
three functions to predict capacity and power consumption:
</p>
<ul>
<li>
A function to predict cooling capacity. The function value needs
to be assigned to <code>capFunT</code>.
</li>
<li>
A function to predict the power input as a function of temperature.
The function value needs to be assigned to <code>EIRFunT</code>.
</li>
<li>
A function to predict the power input as a function of the part load ratio.
The function value needs to be assigned to <code>EIRFunPLR</code>.
</li>
</ul>
</html>",
      revisions="<html>
<ul>
<li>
March 12, 2015, by Michael Wetter:<br/>
Refactored model to make it once continuously differentiable.
This is for issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/373\">373</a>.
</li>
<li>
Jan. 10, 2011, by Michael Wetter:<br/>
Added input signal to switch chiller off, and changed base class to use a dynamic model.
The change of the base class was required to improve the robustness of the model when the control
is switched on again.
</li>
<li>
Sep. 8, 2010, by Michael Wetter:<br/>
Revised model and included it in the Buildings library.
</li>
<li>
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"));
      end PartialElectric;

      function warnIfPerformanceOutOfBounds
        "Function that checks the performance and writes a warning if it is outside of 0.9 to 1.1"
        input Real x "Argument to be checked";
        input String msg "String to be added to warning message";
        input String curveName "Name of the curve that was tested";
        output Integer retVal
          "0 if x is inside bounds, -1 if it is below bounds, or +1 if it is above bounds";

      algorithm
        if (x > 1.1) then
          retVal :=1;
        elseif ( x < 0.9) then
            retVal :=-1;
        else
          retVal :=0;
        end if;
        if (retVal <> 0) then
          Modelica.Utilities.Streams.print(
      "*** Warning: Chiller performance curves at nominal conditions are outside of bounds.
             "       + msg + " is outside of bounds 0.9 to 1.1.
             The value of the curve fit is "       + String(x) + "
             Check the coefficients of the function "       + curveName + ".");
        end if;

      annotation (
          Documentation(info="<html>
<p>
This function checks if the numeric argument is outside of the
interval <i>0.9</i> to <i>1.1</i>.
If this is the case, the function writes a warning.
</p>
</html>",       revisions="<html>
<ul>
<li>
September 12, 2010 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
      end warnIfPerformanceOutOfBounds;

      model FourPortHeatMassExchanger2
        "Model transporting two fluid streams between four ports with storing mass or energy"
        extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
          port_a1(h_outflow(start=h1_outflow_start)),
          port_b1(h_outflow(start=h1_outflow_start)),
          port_a2(h_outflow(start=h2_outflow_start)),
          port_b2(h_outflow(start=h2_outflow_start)));
        extends Buildings.Fluid.Interfaces.FourPortFlowResistanceParameters(
           final computeFlowResistance1=true, final computeFlowResistance2=true);

        parameter Modelica.SIunits.Time tau1 = 30 "Time constant at nominal flow"
           annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
        parameter Modelica.SIunits.Time tau2 = 30 "Time constant at nominal flow"
           annotation (Dialog(tab = "Dynamics", group="Nominal condition"));

        // Advanced
        parameter Boolean homotopyInitialization = true "= true, use homotopy method"
          annotation(Evaluate=true, Dialog(tab="Advanced"));

        // Assumptions
        parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
          "Type of energy balance: dynamic (3 initialization options) or steady state"
          annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
        parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
          "Type of mass balance: dynamic (3 initialization options) or steady state"
          annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

        // Initialization
        parameter Medium1.AbsolutePressure p1_start = Medium1.p_default
          "Start value of pressure"
          annotation(Dialog(tab = "Initialization", group = "Medium 1"));
        parameter Medium1.Temperature T1_start = Medium1.T_default
          "Start value of temperature"
          annotation(Dialog(tab = "Initialization", group = "Medium 1"));
        parameter Medium1.MassFraction X1_start[Medium1.nX] = Medium1.X_default
          "Start value of mass fractions m_i/m"
          annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nXi > 0));
        parameter Medium1.ExtraProperty C1_start[Medium1.nC](
          final quantity=Medium1.extraPropertiesNames)=fill(0, Medium1.nC)
          "Start value of trace substances"
          annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nC > 0));
        parameter Medium1.ExtraProperty C1_nominal[Medium1.nC](
          final quantity=Medium1.extraPropertiesNames) = fill(1E-2, Medium1.nC)
          "Nominal value of trace substances. (Set to typical order of magnitude.)"
         annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nC > 0));

        parameter Medium2.AbsolutePressure p2_start = Medium2.p_default
          "Start value of pressure"
          annotation(Dialog(tab = "Initialization", group = "Medium 2"));
        parameter Medium2.Temperature T2_start = Medium2.T_default
          "Start value of temperature"
          annotation(Dialog(tab = "Initialization", group = "Medium 2"));
        parameter Medium2.MassFraction X2_start[Medium2.nX] = Medium2.X_default
          "Start value of mass fractions m_i/m"
          annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nXi > 0));
        parameter Medium2.ExtraProperty C2_start[Medium2.nC](
          final quantity=Medium2.extraPropertiesNames)=fill(0, Medium2.nC)
          "Start value of trace substances"
          annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nC > 0));
        parameter Medium2.ExtraProperty C2_nominal[Medium2.nC](
          final quantity=Medium2.extraPropertiesNames) = fill(1E-2, Medium2.nC)
          "Nominal value of trace substances. (Set to typical order of magnitude.)"
         annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nC > 0));

        Modelica.SIunits.HeatFlowRate Q1_flow = vol1.heatPort.Q_flow
          "Heat flow rate into medium 1";
        Modelica.SIunits.HeatFlowRate Q2_flow = vol2.heatPort.Q_flow
          "Heat flow rate into medium 2";

        replaceable Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort vol1(nPorts=2)
          constrainedby
          Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort(
              redeclare final package Medium = Medium1,
              nPorts = 2,
              V=m1_flow_nominal*tau1/rho1_nominal,
              final allowFlowReversal=allowFlowReversal1,
              final m_flow_nominal=m1_flow_nominal,
              energyDynamics=if tau1 > Modelica.Constants.eps
                               then energyDynamics else
                               Modelica.Fluid.Types.Dynamics.SteadyState,
              massDynamics=if tau1 > Modelica.Constants.eps
                               then massDynamics else
                               Modelica.Fluid.Types.Dynamics.SteadyState,
              final p_start=p1_start,
              final T_start=T1_start,
              final X_start=X1_start,
              final C_start=C1_start,
              final C_nominal=C1_nominal,
              mSenFac=1) "Volume for fluid 1"
          annotation (Placement(transformation(extent={{-10,70}, {10,50}})));

        replaceable Buildings.Fluid.MixingVolumes.MixingVolume vol2
          constrainedby
          Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort(
              redeclare final package Medium = Medium2,
              nPorts = 2,
              V=m2_flow_nominal*tau2/rho2_nominal,
              final allowFlowReversal=allowFlowReversal2,
              mSenFac=1,
              final m_flow_nominal = m2_flow_nominal,
              energyDynamics=if tau2 > Modelica.Constants.eps
                               then energyDynamics else
                               Modelica.Fluid.Types.Dynamics.SteadyState,
              massDynamics=if tau2 > Modelica.Constants.eps
                               then massDynamics else
                               Modelica.Fluid.Types.Dynamics.SteadyState,
              final p_start=p2_start,
              final T_start=T2_start,
              final X_start=X2_start,
              final C_start=C2_start,
              final C_nominal=C2_nominal) "Volume for fluid 2"
         annotation (Placement(transformation(
              origin={2,-60},
              extent={{-10,10},{10,-10}},
              rotation=180)));

        Buildings.Fluid.FixedResistances.PressureDrop preDro1(
          redeclare final package Medium = Medium1,
          final m_flow_nominal=m1_flow_nominal,
          final deltaM=deltaM1,
          final allowFlowReversal=allowFlowReversal1,
          final show_T=false,
          final from_dp=from_dp1,
          final linearized=linearizeFlowResistance1,
          final homotopyInitialization=homotopyInitialization,
          final dp_nominal=dp1_nominal) "Flow resistance of fluid 1"
          annotation (Placement(transformation(extent={{-76,68},{-56,88}})));

        Buildings.Fluid.FixedResistances.PressureDrop preDro2(
          redeclare final package Medium = Medium2,
          final m_flow_nominal=m2_flow_nominal,
          final deltaM=deltaM2,
          final allowFlowReversal=allowFlowReversal2,
          final show_T=false,
          final from_dp=from_dp2,
          final linearized=linearizeFlowResistance2,
          final homotopyInitialization=homotopyInitialization,
          final dp_nominal=dp2_nominal) "Flow resistance of fluid 2"
          annotation (Placement(transformation(extent={{80,-90},{60,-70}})));

        Buildings.Fluid.Sensors.TemperatureTwoPort TAirConInt(redeclare package
            Medium = Medium1, m_flow_nominal=m1_flow_nominal)
          "Temperature of air into condensor" annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=0,
              origin={-32,78})));
      protected
        parameter Medium1.ThermodynamicState sta1_nominal=Medium1.setState_pTX(
            T=Medium1.T_default, p=Medium1.p_default, X=Medium1.X_default);
        parameter Modelica.SIunits.Density rho1_nominal=Medium1.density(sta1_nominal)
          "Density, used to compute fluid volume";
        parameter Medium2.ThermodynamicState sta2_nominal=Medium2.setState_pTX(
            T=Medium2.T_default, p=Medium2.p_default, X=Medium2.X_default);
        parameter Modelica.SIunits.Density rho2_nominal=Medium2.density(sta2_nominal)
          "Density, used to compute fluid volume";

        parameter Medium1.ThermodynamicState sta1_start=Medium1.setState_pTX(
            T=T1_start, p=p1_start, X=X1_start);
        parameter Modelica.SIunits.SpecificEnthalpy h1_outflow_start = Medium1.specificEnthalpy(sta1_start)
          "Start value for outflowing enthalpy";
        parameter Medium2.ThermodynamicState sta2_start=Medium2.setState_pTX(
            T=T2_start, p=p2_start, X=X2_start);
        parameter Modelica.SIunits.SpecificEnthalpy h2_outflow_start = Medium2.specificEnthalpy(sta2_start)
          "Start value for outflowing enthalpy";

      initial equation
        // Check for tau1
        assert((energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                tau1 > Modelica.Constants.eps,
      "The parameter tau1, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau1 = "       + String(tau1) + "\n");
        assert((massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                tau1 > Modelica.Constants.eps,
      "The parameter tau1, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau1 = "       + String(tau1) + "\n");

       // Check for tau2
        assert((energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                tau2 > Modelica.Constants.eps,
      "The parameter tau2, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau2 = "       + String(tau2) + "\n");
        assert((massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                tau2 > Modelica.Constants.eps,
      "The parameter tau2, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau2 = "       + String(tau2) + "\n");

      equation
        connect(vol1.ports[1], port_b1) annotation (Line(
            points={{-2,70},{20,70},{20,60},{100,60}},
            color={0,127,255}));
        connect(vol2.ports[2], port_b2) annotation (Line(
            points={{2,-70},{-30,-70},{-30,-60},{-100,-60}},
            color={0,127,255}));
        connect(port_a1, preDro1.port_a) annotation (Line(
            points={{-100,60},{-90,60},{-90,78},{-76,78}},
            color={0,127,255}));
        connect(port_a2, preDro2.port_a) annotation (Line(
            points={{100,-60},{90,-60},{90,-80},{80,-80}},
            color={0,127,255}));
        connect(preDro2.port_b, vol2.ports[1]) annotation (Line(
            points={{60,-80},{2,-80},{2,-70}},
            color={0,127,255}));
        connect(preDro1.port_b, TAirConInt.port_a)
          annotation (Line(points={{-56,78},{-42,78}}, color={0,127,255}));
        connect(TAirConInt.port_b, vol1.ports[2])
          annotation (Line(points={{-22,78},{2,78},{2,70}}, color={0,127,255}));
        annotation (
          Documentation(info="<html>
<p>
This component transports two fluid streams between four ports.
It provides the basic model for implementing a dynamic heat exchanger.
</p>
<p>
The model can be used as-is, although there will be no heat or mass transfer
between the two fluid streams.
To add heat transfer, heat flow can be added to the heat port of the two volumes.
See for example
<a href=\"Buildings.Fluid.Chillers.Carnot_y\">
Buildings.Fluid.Chillers.Carnot_y</a>.
To add moisture input into (or moisture output from) volume <code>vol2</code>,
the model can be replaced with
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolumeMoistAir\">
Buildings.Fluid.MixingVolumes.MixingVolumeMoistAir</a>.
</p>
<h4>Implementation</h4>
<p>
The variable names follow the conventions used in
<a href=\"modelica://Modelica.Fluid.Examples.HeatExchanger.BaseClasses.BasicHX\">
Modelica.Fluid.Examples.HeatExchanger.BaseClasses.BasicHX</a>.
</p>
</html>",       revisions="<html>
<ul>
<li>
October 23, 2017, by Michael Wetter:<br/>
Made volume <code>vol1</code> replaceable. This is required for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1013\">Buildings, issue 1013</a>.
</li>
<li>
December 1, 2016, by Michael Wetter:<br/>
Updated model as <code>use_dh</code> is no longer a parameter in the pressure drop model.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/480\">#480</a>.
</li>
<li>
April 11, 2016 by Michael Wetter:<br/>
Corrected wrong hyperlink in documentation for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/450\">issue 450</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Set <code>quantity</code> attributes.
</li>
<li>
November 13, 2015, by Michael Wetter:<br/>
Changed assignments of start values in <code>extends</code> statement.
This is for issue
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/299\">#299</a>.
</li>
<li>
June 2, 2015, by Filip Jorissen:<br/>
Removed final modifier from <code>mSenFac</code> in
<code>vol1</code> and <code>vol2</code>.
This is for issue
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/258=\">#258</a>.
</li>
<li>
May 6, 2015, by Michael Wetter:<br/>
Added missing propagation of <code>allowFlowReversal</code> to
instances <code>vol1</code> and <code>vol2</code>.
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/412\">#412</a>.
</li>
<li>
October 6, 2014, by Michael Wetter:<br/>
Changed medium declaration in pressure drop elements to be final.
</li>
<li>
May 28, 2014, by Michael Wetter:<br/>
Removed <code>annotation(Evaluate=true)</code> for parameters <code>tau1</code>
and <code>tau2</code>.
This is needed to allow changing the time constant after translation.
</li>
<li>
November 12, 2013, by Michael Wetter:<br/>
Removed <code>import Modelica.Constants</code> statement.
</li>
<li>
October 8, 2013, by Michael Wetter:<br/>
Removed parameter <code>show_V_flow</code>.
</li>
<li>
September 26, 2013, by Michael Wetter:<br/>
Removed unrequired <code>sum</code> operator.
</li>
<li>
February 6, 2012, by Michael Wetter:<br/>
Updated documentation.
</li>
<li>
February 3, 2012, by Michael Wetter:<br/>
Removed assignment of <code>m_flow_small</code> as it is no
longer used in its base class.
</li>
<li>
July 29, 2011, by Michael Wetter:
<ul>
<li>
Changed values of
<code>h_outflow_a1_start</code>,
<code>h_outflow_b1_start</code>,
<code>h_outflow_a2_start</code> and
<code>h_outflow_b2_start</code>, and
declared them as final.
</li>
<li>
Set nominal values for <code>vol1.C</code> and <code>vol2.C</code>.
</li>
</ul>
</li>
<li>
July 11, 2011, by Michael Wetter:<br/>
Changed parameterization of fluid volume so that steady-state balance is
used when <code>tau = 0</code>.
</li>
<li>
March 25, 2011, by Michael Wetter:<br/>
Added homotopy operator.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
Added model to compute flow friction.
</li>
<li>
September 10, 2008 by Michael Wetter:<br/>
Added <code>stateSelect=StateSelect.always</code> for temperature of volume 1.
</li>
<li>
Changed temperature sensor from Celsius to Kelvin.
Unit conversion should be made during output
processing.
</li>
<li>
August 5, 2008, by Michael Wetter:<br/>
Replaced instances of <code>Delays.DelayFirstOrder</code> with instances of
<code>MixingVolumes.MixingVolume</code>. This allows to extract liquid for a condensing cooling
coil model.
</li>
<li>
March 25, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={1,1}), graphics={
              Rectangle(
                extent={{-70,80},{70,-80}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-99,64},{102,54}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-99,-56},{102,-66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid)}));
      end FourPortHeatMassExchanger2;
    annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Chillers\">Buildings.Fluid.Chillers</a>.
</p>
</html>"));
    end BaseClassesChiller;

    package Task2c_ed
      model ElectricEIRVFan
        "Chiller Electric EIR with Water - Air Cooling Coil"
        extends Modelica.Icons.Example;
        extends Task2_3.Task2c.Task2c_ed.BaseClasses.PartialElectric(
          P_nominal=-per.QEva_flow_nominal/per.COP_nominal,
          mEva_flow_nominal=per.mEva_flow_nominal,
          mCon_flow_nominal=per.mCon_flow_nominal,
          TSetChi(k=TsetChi),
          sou1(redeclare package Medium = Medium1, m_flow=1.2*76.1));

        replaceable package MediumA = Buildings.Media.Air "Medium model";
        replaceable package MediumW = Buildings.Media.Water "Medium model";
        parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal=16 "Nominal mass flow rate at fan";
        parameter Modelica.SIunits.MassFlowRate mCHW_flow_nominal=16 "Nominal mass flow rate at chilled water";
        parameter Modelica.SIunits.MassFlowRate m_flow_nominal = mCHW_flow_nominal
          "Nominal mass flow rate";
        parameter Modelica.SIunits.Temperature TsetChi=280.15 "Chilled water set point";
        parameter Modelica.SIunits.Temperature TAirSet=286.15 "Cooled air set point";
         parameter Real minSpeFan = 0.01 "Min fan speed load";
        parameter Task2_3.ChillerData.ElectricEIRChiller_30XA220 per
          "Chiller performance data"
          annotation (Placement(transformation(extent={{70,76},{94,100}})));

        Task2_3.Task2b.ElectricEIRCW Chiller(
          redeclare package Medium2 = Medium2,
          per=per,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          dp1_nominal=6000,
          dp2_nominal=6000,
          redeclare package Medium1 = Medium1) "Chiller model"
          annotation (Placement(transformation(extent={{-2,36},{18,56}})));

        Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaData(filNam=
              "C:/Users/BT/OneDrive - University College London/Engineered Environmental Elements/3E COURSEWORK/modelica-buildings-master/Task/ESP_Barcelona.081810_SWEC.mos")
          annotation (Placement(transformation(extent={{130,-84},{110,-64}})));
        Modelica.Blocks.Sources.Constant mPumpFlo1(k=mCHW_flow_nominal)
          "Mass flow rate of pump"
          annotation (Placement(transformation(extent={{90,20},{74,36}})));
        Buildings.Fluid.Sources.FixedBoundary sinA(
          nPorts=1,
          redeclare package Medium = MediumA,
          use_T=true)
          annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TAirOut(m_flow_nominal=
              mAir_flow_nominal, redeclare package Medium = MediumA)
          "Temperature of air leaving cooling coil" annotation (Placement(
              transformation(
              extent={{10,10},{-10,-10}},
              rotation=0,
              origin={-40,-56})));
        Buildings.Fluid.Sources.Outside Weather_Data(redeclare package Medium =
              MediumA, nPorts=1)
          annotation (Placement(transformation(extent={{96,-66},{76,-46}})));
        Buildings.Fluid.Movers.FlowControlled_m_flow ChillerPump(
          redeclare package Medium = Medium2,
          nominalValuesDefineDefaultPressureCurve=true,
          m_flow_nominal=mCHW_flow_nominal,
          dp_nominal=179352)     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={38,10})));
        Buildings.Fluid.Storage.ExpansionVessel expVesChi(V_start=1, redeclare
            package Medium = Medium2)
          annotation (Placement(transformation(extent={{-26,-34},{-12,-21}})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_OUT(m_flow_nominal=
              mCHW_flow_nominal, redeclare package Medium = Medium2)
          "Temperature of water leaving chiller " annotation (Placement(
              transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-32,4})));
        Buildings.Fluid.Sensors.TemperatureTwoPort TCHW_IN(m_flow_nominal=
              mCHW_flow_nominal, redeclare package Medium = Medium2)
          "Temperature of water returning to chiller " annotation (Placement(
              transformation(
              extent={{-10,10},{10,-10}},
              rotation=90,
              origin={38,-20})));
        Buildings.Controls.Continuous.LimPID
                                   conPID(
          Td=1,
          k=0.5,
          yMin=minSpeFan,
          yMax=50,
          controllerType=Modelica.Blocks.Types.SimpleController.PID,
          Ti=200,
          reverseAction=false)
                annotation (Placement(transformation(extent={{20,-90},{32,-78}})));
        Modelica.Blocks.Sources.Constant TAirOutSet(k=TAirSet) "TAirOut set point "
          annotation (Placement(transformation(extent={{-2,-90},{10,-78}})));
        Buildings.Fluid.Movers.SpeedControlled_y Fan(
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          addPowerToMedium=false,
          use_inputFilter=true,
          redeclare package Medium = MediumA,
          per(pressure(V_flow=mAir_flow_nominal*{0,1,2}/1.2, dp=500*{2,1,0})))
          "Supply air fan"
          annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=0,
              origin={52,-56})));
        Modelica.Blocks.Continuous.Integrator integratorPump
          annotation (Placement(transformation(extent={{76,-6},{90,8}})));
        Modelica.Blocks.Continuous.Integrator integratorFan
          annotation (Placement(transformation(extent={{-10,-74},{-20,-64}})));
        Modelica.Blocks.Continuous.Integrator integratorChi
          annotation (Placement(transformation(extent={{38,70},{52,84}})));
        Buildings.Fluid.HeatExchangers.WetCoilCounterFlow CoolingCoil(
          redeclare package Medium1 = Buildings.Media.Water,
          redeclare package Medium2 = Buildings.Media.Air,
          m1_flow_nominal=32,
          m2_flow_nominal=16,
          dp1_nominal=1000 + 89580,
          dp2_nominal=249*3,
          UA_nominal=mAir_flow_nominal*1006*5)
          annotation (Placement(transformation(extent={{-4,-60},{16,-40}})));
      equation
        connect(Chiller.port_b1, res1.port_a) annotation (Line(
            points={{18,52},{34,52}},
            color={0,127,255},
            smooth=Smooth.None));
        connect(Chiller.on, greaterThreshold.y) annotation (Line(
            points={{-4,49},{-10,49},{-10,86},{-25,86}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(TSetChi.y, Chiller.TSet) annotation (Line(points={{-75,2},{-50,2},
                {-50,43},{-4,43}}, color={0,0,127}));
        connect(TAirOut.port_b,sinA. ports[1])
          annotation (Line(points={{-50,-56},{-70,-56}}, color={0,127,255}));
        connect(Weather_Data.weaBus, weaData.weaBus) annotation (Line(
            points={{96,-55.8},{96,-74},{110,-74}},
            color={255,204,51},
            thickness=0.5));
        connect(ChillerPump.port_b, Chiller.port_a2)
          annotation (Line(points={{38,20},{38,40},{18,40}}, color={0,127,255}));
        connect(ChillerPump.m_flow_in, mPumpFlo1.y) annotation (Line(points={{26,10},{
                16,10},{16,28},{73.2,28}}, color={0,0,127}));
        connect(TCHW_OUT.port_a, Chiller.port_b2) annotation (Line(points={{-32,
                14},{-32,40},{-2,40}}, color={0,127,255}));
        connect(TCHW_IN.port_b, ChillerPump.port_a)
          annotation (Line(points={{38,-10},{38,0}},   color={0,127,255}));
        connect(TAirOutSet.y, conPID.u_s)
          annotation (Line(points={{10.6,-84},{18.8,-84}}, color={0,0,127}));
        connect(TAirOut.T, conPID.u_m) annotation (Line(points={{-40,-67},{-40,-98},{26,
                -98},{26,-91.2}},    color={0,0,127}));
        connect(Fan.port_a, Weather_Data.ports[1])
          annotation (Line(points={{62,-56},{76,-56}}, color={0,127,255}));
        connect(conPID.y,Fan. y) annotation (Line(points={{32.6,-84},{52,-84},{52,-68}},
                                     color={0,0,127}));
        connect(sou1.weaBus, weaData.weaBus) annotation (Line(
            points={{-90,52.2},{-110,52.2},{-110,-102},{102,-102},{102,-74},{110,-74}},
            color={255,204,51},
            thickness=0.5));

        connect(sou1.ports[1], Chiller.port_a1)
          annotation (Line(points={{-70,52},{-2,52}}, color={0,127,255}));
        connect(ChillerPump.P, integratorPump.u) annotation (Line(points={{29,21},{29,
                24},{62,24},{62,1},{74.6,1}}, color={0,0,127}));
        connect(Fan.P, integratorFan.u) annotation (Line(points={{41,-65},{2,-65},{2,-69},
                {-9,-69}}, color={0,0,127}));
        connect(Chiller.P, integratorChi.u) annotation (Line(points={{19,55},{26,
                55},{26,77},{36.6,77}}, color={0,0,127}));
        connect(TAirOut.port_a, CoolingCoil.port_b2)
          annotation (Line(points={{-30,-56},{-4,-56}}, color={0,127,255}));
        connect(Fan.port_b, CoolingCoil.port_a2)
          annotation (Line(points={{42,-56},{16,-56}}, color={0,127,255}));
        connect(TCHW_IN.port_a, CoolingCoil.port_b1)
          annotation (Line(points={{38,-30},{38,-44},{16,-44}}, color={0,127,255}));
        connect(TCHW_OUT.port_b, CoolingCoil.port_a1)
          annotation (Line(points={{-32,-6},{-32,-44},{-4,-44}}, color={0,127,255}));
        connect(expVesChi.port_a, CoolingCoil.port_a1) annotation (Line(points={{-19,-34},
                {-20,-34},{-20,-44},{-4,-44}}, color={0,127,255}));
        annotation (
      experiment(
            StartTime=20044800,
            StopTime=20649600,
            Interval=3600,
            Tolerance=1e-06),
      __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/Chillers/Examples/ElectricEIR.mos"
              "Simulate and plot"),
          Documentation(info="<html>
      <pre>                 
NOTE: when changing chiller model the following parameters should be updated:
1. mCHW_flow_nominal should be changed in the text layer of the main model (e.g ElectricEIRConst), this should be the same value as 
'mEva_flow_nominal'.

2. In the diagram model, double click on 'coocoi', update 'm1_flow_nominal' to be same value as 'mEva_flow_nominal', 
type the value in directly here.

3. In the diagram model, double click on 'sou1', update 'm1_nominal' to be same value as 'mCon_flow_nominal', type the value in 
directly here.

mEva_flow_nominal and mCon_flow_nominal can be found in following location in the information or text layer of the dataset: 
ElementsCWModel3.ChillerData.ElectricEIRCarrier_30RB360, select corresponding chiller.

No changes need to be made to the base classes with these models. Modifications can be made directly in the top layer model.
<pre>

<p>
Example that simulates a chiller whose efficiency is computed based on the
condenser entering and evaporator leaving fluid temperature.
A bicubic polynomial is used to compute the chiller part load performance.

This model fan speed is controlled by a PI controller and modulated to meet the set point of the cooled air on the outlet of the cooling coil.
</p>
</html>",       revisions="<html>
<ul>
<li>
December 10, 2018, by Anneka Kang - changed to air cooled chiller, PID control, added cooling coil and weabus links.
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"),Diagram(coordinateSystem(extent={{-120,-120},{120,120}})),
          Icon(coordinateSystem(extent={{-120,-120},{120,120}})));
      end ElectricEIRVFan;

      package BaseClasses
        "Package with base classes for Buildings.Fluid.Chillers.Examples"
        extends Modelica.Icons.BasesPackage;
        partial model PartialElectric
          "Base class for test model of chiller electric EIR"
         package Medium1 = Buildings.Media.Air "Medium model";
         package Medium2 = Buildings.Media.Water "Medium model";

          parameter Modelica.SIunits.Power P_nominal
            "Nominal compressor power (at y=1)";
          parameter Modelica.SIunits.TemperatureDifference dTEva_nominal=10
            "Temperature difference evaporator inlet-outlet";

          parameter Real COPc_nominal = 3 "Chiller COP";
          parameter Modelica.SIunits.MassFlowRate mEva_flow_nominal
            "Nominal mass flow rate at evaporator";
          parameter Modelica.SIunits.MassFlowRate mCon_flow_nominal
            "Nominal mass flow rate at condenser";

          Buildings.Fluid.Sources.FixedBoundary sin1(
            redeclare package Medium = Medium1,
            nPorts=1)                           annotation (Placement(
                transformation(
                extent={{10,-10},{-10,10}},
                origin={80,52})));
          Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=0.5)
            annotation (Placement(transformation(extent={{-46,76},{-26,96}})));
          Buildings.Fluid.FixedResistances.PressureDrop res1(
            redeclare package Medium = Medium1,
            m_flow_nominal=mCon_flow_nominal,
            dp_nominal=6000) "Flow resistance"
            annotation (Placement(transformation(extent={{34,42},{54,62}})));
          Modelica.Blocks.Sources.Constant TSetChi(k=273.15 + 7)
            "Chiller Water Leaving Temperature Set Point"
            annotation (Placement(transformation(extent={{-96,-8},{-76,12}})));
          Modelica.Blocks.Sources.Constant ChillerON(k=1) "Chiller ON"
            annotation (Placement(transformation(extent={{-98,76},{-78,96}})));
          Buildings.Fluid.Sources.MassFlowSource_WeatherData sou1(nPorts=1,
              redeclare package Medium = Medium1)
            annotation (Placement(transformation(extent={{-90,42},{-70,62}})));
        equation

          connect(res1.port_b, sin1.ports[1]) annotation (Line(
              points={{54,52},{70,52}},
              color={0,127,255},
              smooth=Smooth.None));
          connect(ChillerON.y, greaterThreshold.u)
            annotation (Line(points={{-77,86},{-48,86}}, color={0,0,127}));
        end PartialElectric;
      annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Chillers.Examples\">Buildings.Fluid.Chillers.Examples</a>.
</p>
</html>"));
      end BaseClasses;

      model ElectricEIRCW "Electric chiller based on the DOE-2.1 model"
        extends Task2_3.Task2c.Task2c_ed.BaseClassesChiller.PartialElectric(
          final QEva_flow_nominal=per.QEva_flow_nominal,
          final COP_nominal=per.COP_nominal,
          final PLRMax=per.PLRMax,
          final PLRMinUnl=per.PLRMinUnl,
          final PLRMin=per.PLRMin,
          final etaMotor=per.etaMotor,
          final mEva_flow_nominal=per.mEva_flow_nominal,
          final mCon_flow_nominal=per.mCon_flow_nominal,
          final TEvaLvg_nominal=per.TEvaLvg_nominal);

        parameter Task2_3.ChillerData.ElectricEIRCarrier_30RB360 per
          "Performance data" annotation (choicesAllMatching=true, Placement(
              transformation(extent={{40,80},{60,100}})));

      protected
        final parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC
          TConEnt_nominal_degC=
          Modelica.SIunits.Conversions.to_degC(per.TConEnt_nominal)
          "Temperature of fluid entering condenser at nominal condition";

        Modelica.SIunits.Conversions.NonSIunits.Temperature_degC TConEnt_degC
          "Temperature of fluid entering condenser";
      initial equation
        // Verify correctness of performance curves, and write warning if error is bigger than 10%
        Buildings.Fluid.Chillers.BaseClasses.warnIfPerformanceOutOfBounds(
           Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT,
           x1=TEvaLvg_nominal_degC, x2=TConEnt_nominal_degC),
           "Capacity as function of temperature ",
           "per.capFunT");
      equation
        TConEnt_degC=Modelica.SIunits.Conversions.to_degC(TConEnt);

        if on then
          // Compute the chiller capacity fraction, using a biquadratic curve.
          // Since the regression for capacity can have negative values (for unreasonable temperatures),
          // we constrain its return value to be non-negative. This prevents the solver to pick the
          // unrealistic solution.
          capFunT = Buildings.Utilities.Math.Functions.smoothMax(
             x1 = 1E-6,
             x2 = Buildings.Utilities.Math.Functions.biquadratic(a=per.capFunT, x1=TEvaLvg_degC, x2=TConEnt_degC),
             deltaX = 1E-7);
      /*    assert(capFunT > 0.1, "Error: Received capFunT = " + String(capFunT)  + ".\n"
           + "Coefficient for polynomial seem to be not valid for the encountered temperature range.\n"
           + "Temperatures are TConEnt_degC = " + String(TConEnt_degC) + " degC\n"
           + "                 TEvaLvg_degC = " + String(TEvaLvg_degC) + " degC");
*/
          // Chiller energy input ratio biquadratic curve.
          EIRFunT = Buildings.Utilities.Math.Functions.biquadratic(a=per.EIRFunT, x1=TEvaLvg_degC, x2=TConEnt_degC);
          // Chiller energy input ratio quadratic curve
          EIRFunPLR   = per.EIRFunPLR[1]+per.EIRFunPLR[2]*PLR2+per.EIRFunPLR[3]*PLR2^2;
        else
          capFunT   = 0;
          EIRFunT   = 0;
          EIRFunPLR = 0;
        end if;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                         graphics={
              Rectangle(
                extent={{-99,-54},{102,-66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,-66},{0,-54}},
                lineColor={0,0,127},
                pattern=LinePattern.None,
                fillColor={0,0,127},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-104,66},{98,54}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-2,54},{98,66}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,52},{-40,12}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,70},{58,52}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,2},{-52,12},{-32,12},{-42,2}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-44,-10},{-40,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{38,52},{42,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-56,-50},{58,-68}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{18,24},{62,-18}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{40,24},{22,-8},{58,-8},{40,24}},
                lineColor={0,0,0},
                smooth=Smooth.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}),
      defaultComponentName="chi",
      Documentation(info="<html>
<p>
Model of an electric chiller, based on the DOE-2.1 chiller model and
the EnergyPlus chiller model <code>Chiller:Electric:EIR</code>.
</p>
<p> This model uses three functions to predict capacity and power consumption:
</p>
<ul>
<li>
A biquadratic function is used to predict cooling capacity as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
<li>
A quadratic functions is used to predict power input to cooling capacity ratio with respect to the part load ratio.
</li>
<li>
A biquadratic functions is used to predict power input to cooling capacity ratio as a function of
condenser entering and evaporator leaving fluid temperature.
</li>
</ul>
<p>
These curves are stored in the data record <code>per</code> and are available from
<a href=\"Buildings.Fluid.Chillers.Data.ElectricEIR\">
Buildings.Fluid.Chillers.Data.ElectricEIR</a>.
Additional performance curves can be developed using
two available techniques (Hydeman and Gillespie, 2002). The first technique is called the
Least-squares Linear Regression method and is used when sufficient performance data exist
to employ standard least-square linear regression techniques. The second technique is called
Reference Curve Method and is used when insufficient performance data exist to apply linear
regression techniques. A detailed description of both techniques can be found in
Hydeman and Gillespie (2002).
</p>
<p>
The model takes as an input the set point for the leaving chilled water temperature,
which is met if the chiller has sufficient capacity.
Thus, the model has a built-in, ideal temperature control.
The model has three tests on the part load ratio and the cycling ratio:
</p>
<ol>
<li>
The test<pre>
  PLR1 =min(QEva_flow_set/QEva_flow_ava, per.PLRMax);
</pre>
ensures that the chiller capacity does not exceed the chiller capacity specified
by the parameter <code>per.PLRMax</code>.
</li>
<li>
The test <pre>
  CR = min(PLR1/per.PRLMin, 1.0);
</pre>
computes a cycling ratio. This ratio expresses the fraction of time
that a chiller would run if it were to cycle because its load is smaller than the
minimal load at which it can operate.
Note that this model continuously operates even if the part load ratio is below the minimum part load ratio.
Its leaving evaporator and condenser temperature can therefore be considered as an
average temperature between the modes where the compressor is off and on.
</li>
<li>
The test <pre>
  PLR2 = max(per.PLRMinUnl, PLR1);
</pre>
computes the part load ratio of the compressor.
The assumption is that for a part load ratio below <code>per.PLRMinUnl</code>,
the chiller uses hot gas bypass to reduce the capacity, while the compressor
power draw does not change.
</li>
</ol>
<p>
The electric power only contains the power for the compressor, but not any power for pumps or fans.
</p>
<p>
The model can be parametrized to compute a transient
or steady-state response.
The transient response of the boiler is computed using a first
order differential equation for the evaporator and condenser fluid volumes.
The chiller outlet temperatures are equal to the temperatures of these lumped volumes.
</p>
<h4>References</h4>
<ul>
<li>
Hydeman, M. and K.L. Gillespie. 2002. Tools and Techniques to Calibrate Electric Chiller
Component Models. <i>ASHRAE Transactions</i>, AC-02-9-1.
</li>
</ul>
</html>",
      revisions="<html>
<ul>
<li>
March 12, 2015, by Michael Wetter:<br/>
Refactored model to make it once continuously differentiable.
This is for issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/373\">373</a>.
</li>
<li>
Jan. 9, 2011, by Michael Wetter:<br/>
Added input signal to switch chiller off.
</li>
<li>
Sep. 8, 2010, by Michael Wetter:<br/>
Revised model and included it in the Buildings library.
</li>
<li>
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"));
      end ElectricEIRCW;

      package BaseClassesChiller
        "Package with base classes for Buildings.Fluid.Chillers"
        extends Modelica.Icons.BasesPackage;

        partial model Carnot
          extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
            m1_flow_nominal = QCon_flow_nominal/cp1_default/dTCon_nominal,
            m2_flow_nominal = QEva_flow_nominal/cp2_default/dTEva_nominal);

          parameter Modelica.SIunits.HeatFlowRate QEva_flow_nominal(max=0)
            "Nominal cooling heat flow rate (QEva_flow_nominal < 0)"
            annotation (Dialog(group="Nominal condition"));
          parameter Modelica.SIunits.HeatFlowRate QCon_flow_nominal(min=0)
            "Nominal heating flow rate"
            annotation (Dialog(group="Nominal condition"));

          parameter Modelica.SIunits.TemperatureDifference dTEva_nominal(
            final max=0) = -10 "Temperature difference evaporator outlet-inlet"
            annotation (Dialog(group="Nominal condition"));
          parameter Modelica.SIunits.TemperatureDifference dTCon_nominal(
            final min=0) = 10 "Temperature difference condenser outlet-inlet"
            annotation (Dialog(group="Nominal condition"));

          // Efficiency
          parameter Boolean use_eta_Carnot_nominal = true
            "Set to true to use Carnot effectiveness etaCarnot_nominal rather than COP_nominal"
            annotation(Dialog(group="Efficiency"));
          parameter Real etaCarnot_nominal(unit="1") = COP_nominal/
            (TUseAct_nominal/(TCon_nominal+TAppCon_nominal - (TEva_nominal-TAppEva_nominal)))
            "Carnot effectiveness (=COP/COP_Carnot) used if use_eta_Carnot_nominal = true"
            annotation (Dialog(group="Efficiency", enable=use_eta_Carnot_nominal));

          parameter Real COP_nominal(unit="1") = etaCarnot_nominal*TUseAct_nominal/
            (TCon_nominal+TAppCon_nominal - (TEva_nominal-TAppEva_nominal))
            "Coefficient of performance at TEva_nominal and TCon_nominal, used if use_eta_Carnot_nominal = false"
            annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

          parameter Modelica.SIunits.Temperature TCon_nominal = 303.15
            "Condenser temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
            annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));
          parameter Modelica.SIunits.Temperature TEva_nominal = 278.15
            "Evaporator temperature used to compute COP_nominal if use_eta_Carnot_nominal=false"
            annotation (Dialog(group="Efficiency", enable=not use_eta_Carnot_nominal));

          parameter Real a[:] = {1}
            "Coefficients for efficiency curve (need p(a=a, yPL=1)=1)"
            annotation (Dialog(group="Efficiency"));

          parameter Modelica.SIunits.Pressure dp1_nominal(displayUnit="Pa")
            "Pressure difference over condenser"
            annotation (Dialog(group="Nominal condition"));
          parameter Modelica.SIunits.Pressure dp2_nominal(displayUnit="Pa")
            "Pressure difference over evaporator"
            annotation (Dialog(group="Nominal condition"));

          parameter Modelica.SIunits.TemperatureDifference TAppCon_nominal(min=0) = if cp1_default < 1500 then 5 else 2
            "Temperature difference between refrigerant and working fluid outlet in condenser"
            annotation (Dialog(group="Efficiency"));

          parameter Modelica.SIunits.TemperatureDifference TAppEva_nominal(min=0) = if cp2_default < 1500 then 5 else 2
            "Temperature difference between refrigerant and working fluid outlet in evaporator"
            annotation (Dialog(group="Efficiency"));

          parameter Boolean homotopyInitialization=true "= true, use homotopy method"
            annotation (Dialog(tab="Advanced"));

          parameter Boolean from_dp1=false
            "= true, use m_flow = f(dp) else dp = f(m_flow)"
            annotation (Dialog(tab="Flow resistance", group="Condenser"));
          parameter Boolean from_dp2=false
            "= true, use m_flow = f(dp) else dp = f(m_flow)"
            annotation (Dialog(tab="Flow resistance", group="Evaporator"));

          parameter Boolean linearizeFlowResistance1=false
            "= true, use linear relation between m_flow and dp for any flow rate"
            annotation (Dialog(tab="Flow resistance", group="Condenser"));
          parameter Boolean linearizeFlowResistance2=false
            "= true, use linear relation between m_flow and dp for any flow rate"
            annotation (Dialog(tab="Flow resistance", group="Evaporator"));

          parameter Real deltaM1(final unit="1")=0.1
            "Fraction of nominal flow rate where flow transitions to laminar"
            annotation (Dialog(tab="Flow resistance", group="Condenser"));
          parameter Real deltaM2(final unit="1")=0.1
            "Fraction of nominal flow rate where flow transitions to laminar"
            annotation (Dialog(tab="Flow resistance", group="Evaporator"));

          parameter Modelica.SIunits.Time tau1=60
            "Time constant at nominal flow rate (used if energyDynamics1 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
            annotation (Dialog(tab="Dynamics", group="Condenser"));
          parameter Modelica.SIunits.Time tau2=60
            "Time constant at nominal flow rate (used if energyDynamics2 <> Modelica.Fluid.Types.Dynamics.SteadyState)"
            annotation (Dialog(tab="Dynamics", group="Evaporator"));

          parameter Modelica.SIunits.Temperature T1_start=Medium1.T_default
            "Initial or guess value of set point"
            annotation (Dialog(tab="Dynamics", group="Condenser"));
          parameter Modelica.SIunits.Temperature T2_start=Medium2.T_default
            "Initial or guess value of set point"
            annotation (Dialog(tab="Dynamics", group="Evaporator"));

          parameter Modelica.Fluid.Types.Dynamics energyDynamics=
            Modelica.Fluid.Types.Dynamics.SteadyState "Type of energy balance: dynamic (3 initialization options) or steady state"
            annotation (Dialog(tab="Dynamics", group="Evaporator and condenser"));

          Modelica.Blocks.Interfaces.RealOutput QCon_flow(
            final quantity="HeatFlowRate",
            final unit="W") "Actual heating heat flow rate added to fluid 1"
            annotation (Placement(transformation(extent={{100,80},{120,100}}),
                iconTransformation(extent={{100,80},{120,100}})));

          Modelica.Blocks.Interfaces.RealOutput P(
            final quantity="Power",
            final unit="W") "Electric power consumed by compressor"
            annotation (Placement(transformation(extent={{100,-10},{120,10}}),
                iconTransformation(extent={{100,-10},{120,10}})));

          Modelica.Blocks.Interfaces.RealOutput QEva_flow(
            final quantity="HeatFlowRate",
            final unit="W") "Actual cooling heat flow rate removed from fluid 2"
            annotation (Placement(transformation(extent={{100,-100},{120,-80}}),
                iconTransformation(extent={{100,-100},{120,-80}})));

          Real yPL(final unit="1", min=0) = if COP_is_for_cooling
             then QEva_flow/QEva_flow_nominal
             else QCon_flow/QCon_flow_nominal "Part load ratio";

          Real etaPL(final unit = "1")=
            if evaluate_etaPL
              then 1
            else Buildings.Utilities.Math.Functions.polynomial(a=a, x=yPL)
            "Efficiency due to part load (etaPL(yPL=1)=1)";

          Real COP(min=0, final unit="1") = etaCarnot_nominal_internal * COPCar * etaPL
            "Coefficient of performance";

          Real COPCar(min=0) = TUseAct/Buildings.Utilities.Math.Functions.smoothMax(
            x1=1,
            x2=TConAct - TEvaAct,
            deltaX=0.25) "Carnot efficiency";

          Modelica.SIunits.Temperature TConAct(start=TCon_nominal + TAppCon_nominal)=
            Medium1.temperature(staB1) + QCon_flow/QCon_flow_nominal*TAppCon_nominal
            "Condenser temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";

          Modelica.SIunits.Temperature TEvaAct(start=TEva_nominal - TAppEva_nominal)=
            Medium2.temperature(staB2) - QEva_flow/QEva_flow_nominal*TAppEva_nominal
            "Evaporator temperature used to compute efficiency, taking into account pinch temperature between fluid and refrigerant";

        protected
          constant Boolean COP_is_for_cooling
            "Set to true if the specified COP is for cooling";

          parameter Real etaCarnot_nominal_internal(unit="1")=
            if use_eta_Carnot_nominal
              then etaCarnot_nominal
              else COP_nominal/
                   (TUseAct_nominal / (TCon_nominal + TAppCon_nominal - (TEva_nominal - TAppEva_nominal)))
            "Carnot effectiveness (=COP/COP_Carnot) used to compute COP";

          // For Carnot_y, computing etaPL = f(yPL) introduces a nonlinear equation.
          // The parameter below avoids this if a = {1}.
          final parameter Boolean evaluate_etaPL=
            (size(a, 1) == 1 and abs(a[1] - 1)  < Modelica.Constants.eps)
            "Flag, true if etaPL should be computed as it depends on yPL"
            annotation(Evaluate=true);

          final parameter Modelica.SIunits.Temperature TUseAct_nominal=
            if COP_is_for_cooling
              then TEva_nominal - TAppEva_nominal
              else TCon_nominal + TAppCon_nominal
            "Nominal evaporator temperature for chiller or condenser temperature for heat pump, taking into account pinch temperature between fluid and refrigerant";
          Modelica.SIunits.Temperature TUseAct=if COP_is_for_cooling then TEvaAct else TConAct
            "Temperature of useful heat (evaporator for chiller, condenser for heat pump), taking into account pinch temperature between fluid and refrigerant";

          final parameter Modelica.SIunits.SpecificHeatCapacity cp1_default=
            Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
              p = Medium1.p_default,
              T = Medium1.T_default,
              X = Medium1.X_default))
            "Specific heat capacity of medium 1 at default medium state";

          final parameter Modelica.SIunits.SpecificHeatCapacity cp2_default=
            Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
              p = Medium2.p_default,
              T = Medium2.T_default,
              X = Medium2.X_default))
            "Specific heat capacity of medium 2 at default medium state";

          Medium1.ThermodynamicState staA1 = Medium1.setState_phX(
            port_a1.p,
            inStream(port_a1.h_outflow),
            inStream(port_a1.Xi_outflow)) "Medium properties in port_a1";
          Medium1.ThermodynamicState staB1 = Medium1.setState_phX(
            port_b1.p,
            port_b1.h_outflow,
            port_b1.Xi_outflow) "Medium properties in port_b1";
          Medium2.ThermodynamicState staA2 = Medium2.setState_phX(
            port_a2.p,
            inStream(port_a2.h_outflow),
            inStream(port_a2.Xi_outflow)) "Medium properties in port_a2";
          Medium2.ThermodynamicState staB2 = Medium2.setState_phX(
            port_b2.p,
            port_b2.h_outflow,
            port_b2.Xi_outflow) "Medium properties in port_b2";

          replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface con
            constrainedby Buildings.Fluid.Interfaces.PartialTwoPortInterface(
            redeclare final package Medium = Medium1,
            final allowFlowReversal=allowFlowReversal1,
            final m_flow_nominal=m1_flow_nominal,
            final m_flow_small=m1_flow_small,
            final show_T=false) "Condenser"
            annotation (Placement(transformation(extent={{-10,50},{10,70}})));

          replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface eva
            constrainedby Buildings.Fluid.Interfaces.PartialTwoPortInterface(
            redeclare final package Medium = Medium2,
            final allowFlowReversal=allowFlowReversal2,
            final m_flow_nominal=m2_flow_nominal,
            final m_flow_small=m2_flow_small,
            final show_T=false) "Evaporator"
            annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));

        initial equation
          assert(dTEva_nominal < 0,
            "Parameter dTEva_nominal must be negative.");
          assert(dTCon_nominal > 0,
            "Parameter dTCon_nominal must be positive.");

          assert(abs(Buildings.Utilities.Math.Functions.polynomial(
                 a=a, x=1)-1) < 0.01, "Efficiency curve is wrong. Need etaPL(y=1)=1.");
          assert(etaCarnot_nominal_internal < 1,   "Parameters lead to etaCarnot_nominal > 1. Check parameters.");

        equation
          connect(port_a2, eva.port_a)
            annotation (Line(points={{100,-60},{56,-60},{10,-60}}, color={0,127,255}));
          connect(eva.port_b, port_b2) annotation (Line(points={{-10,-60},{-100,-60}},
                         color={0,127,255}));
          connect(port_a1, con.port_a)
            annotation (Line(points={{-100,60},{-56,60},{-10,60}}, color={0,127,255}));
          connect(con.port_b, port_b1)
            annotation (Line(points={{10,60},{56,60},{100,60}}, color={0,127,255}));

          annotation (
          Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},
                    {100,100}}),       graphics={
                Rectangle(
                  extent={{-70,80},{70,-80}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-56,68},{58,50}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-56,-52},{58,-70}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-103,64},{98,54}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,54},{98,64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-101,-56},{100,-66}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-100,-66},{0,-56}},
                  lineColor={0,0,127},
                  pattern=LinePattern.None,
                  fillColor={0,0,127},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,0},{-52,-12},{-32,-12},{-42,0}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,0},{-52,10},{-32,10},{-42,0}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,50},{-40,10}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,-12},{-40,-52}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{38,50},{42,-52}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{18,22},{62,-20}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{40,22},{22,-10},{58,-10},{40,22}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Line(points={{0,68},{0,90},{90,90},{100,90}},
                                                         color={0,0,255}),
                Line(points={{0,-70},{0,-90},{100,-90}}, color={0,0,255}),
                Line(points={{62,0},{100,0}},                 color={0,0,255})}),
              Documentation(info="<html>
<p>
This is the base class for the Carnot chiller and the Carnot heat pump
whose coefficient of performance COP changes
with temperatures in the same way as the Carnot efficiency changes.
</p>
<p>
The model allows to either specify the Carnot effectivness
<i>&eta;<sub>Carnot,0</sub></i>, or
a <i>COP<sub>0</sub></i>
at the nominal conditions, together with
the evaporator temperature <i>T<sub>eva,0</sub></i> and
the condenser temperature <i>T<sub>con,0</sub></i>, in which
case the model computes the Carnot effectivness as
</p>
<p align=\"center\" style=\"font-style:italic;\">
&eta;<sub>Carnot,0</sub> =
  COP<sub>0</sub>
&frasl;  (T<sub>use,0</sub> &frasl; (T<sub>con,0</sub>-T<sub>eva,0</sub>)),
</p>
<p>
where
<i>T<sub>use</sub></i> is the temperature of the the useful heat,
e.g., the evaporator temperature for a chiller or the condenser temperature
for a heat pump.
</p>
<p>
The COP is computed as the product
</p>
<p align=\"center\" style=\"font-style:italic;\">
  COP = &eta;<sub>Carnot,0</sub> COP<sub>Carnot</sub> &eta;<sub>PL</sub>,
</p>
<p>
where <i>COP<sub>Carnot</sub></i> is the Carnot efficiency and
<i>&eta;<sub>PL</sub></i> is the part load efficiency, expressed using
a polynomial.
This polynomial has the form
</p>
<p align=\"center\" style=\"font-style:italic;\">
  &eta;<sub>PL</sub> = a<sub>1</sub> + a<sub>2</sub> y + a<sub>3</sub> y<sup>2</sup> + ...
</p>
<p>
where <i>y &isin; [0, 1]</i> is
either the part load for cooling in case of a chiller, or the part load of heating in
case of a heat pump, and the coefficients <i>a<sub>i</sub></i>
are declared by the parameter <code>a</code>.
</p>
<h4>Implementation</h4>
<p>
To make this base class applicable to chiller or heat pumps, it uses
the boolean constant <code>COP_is_for_cooling</code>.
Depending on its value, the equations for the coefficient of performance
and the part load ratio are set up.
</p>
</html>",         revisions="<html>
<ul>
<li>
June 16, 2017, by Michael Wetter:<br/>
Added temperature difference between fluids in condenser and evaporator
for computation of nominal COP and effectiveness.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
March 28, 2017, by Felix Buenning:<br/>
Added temperature difference between fluids in condenser and evaporator.
The difference is based on discussions with Emerson Climate Technologies.<br/>
This is for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/698\">
#698</a>.
</li>
<li>
January 2, 2017, by Filip Jorissen:<br/>
Removed option for choosing what temperature
should be used to compute the Carnot efficiency.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/497\">
issue 497</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
First implementation of this base class.
</li>
</ul>
</html>"));
        end Carnot;

        partial model PartialCarnot_T
          "Partial model for chiller with performance curve adjusted based on Carnot efficiency"
          extends Task2_3.Task2c.Task2c_ed.BaseClassesChiller.Carnot;

        protected
          Modelica.Blocks.Sources.RealExpression PEle "Electrical power consumption"
            annotation (Placement(transformation(extent={{40,-10},{60,10}})));

        equation
          connect(PEle.y, P)
            annotation (Line(points={{61,0},{110,0},{110,0}}, color={0,0,127}));
          annotation (
        Documentation(info="<html>
<p>
This is a partial model of a chiller whose coefficient of performance (COP) changes
with temperatures in the same way as the Carnot efficiency changes.
This base class is used for the Carnot chiller and Carnot heat pump
that uses the compressor part load ratio as the control signal.
</p>
</html>",
        revisions="<html>
<ul>
<li>
January 26, 2016, by Michael Wetter:<br/>
First implementation of this base class.
</li>
</ul>
</html>"));
        end PartialCarnot_T;

        partial model PartialCarnot_y
          "Partial chiller model with performance curve adjusted based on Carnot efficiency"
          extends Carnot(
            final QCon_flow_nominal= P_nominal - QEva_flow_nominal,
            final QEva_flow_nominal = if COP_is_for_cooling
                                      then -P_nominal * COP_nominal
                                      else -P_nominal * (COP_nominal-1),
            redeclare Buildings.Fluid.HeatExchangers.HeaterCooler_u con(
              final from_dp=from_dp1,
              final dp_nominal=dp1_nominal,
              final linearizeFlowResistance=linearizeFlowResistance1,
              final deltaM=deltaM1,
              final tau=tau1,
              final T_start=T1_start,
              final energyDynamics=energyDynamics,
              final massDynamics=energyDynamics,
              final homotopyInitialization=homotopyInitialization,
              final Q_flow_nominal=QCon_flow_nominal),
              redeclare Buildings.Fluid.HeatExchangers.HeaterCooler_u eva(
              final from_dp=from_dp2,
              final dp_nominal=dp2_nominal,
              final linearizeFlowResistance=linearizeFlowResistance2,
              final deltaM=deltaM2,
              final tau=tau2,
              final T_start=T2_start,
              final energyDynamics=energyDynamics,
              final massDynamics=energyDynamics,
              final homotopyInitialization=homotopyInitialization,
              final Q_flow_nominal=QEva_flow_nominal));

          parameter Modelica.SIunits.Power P_nominal(min=0)
            "Nominal compressor power (at y=1)"
            annotation (Dialog(group="Nominal condition"));

          Modelica.Blocks.Interfaces.RealInput y(min=0, max=1, unit="1")
            "Part load ratio of compressor"
            annotation (Placement(transformation(extent={{-140,70},{-100,110}})));

        protected
          Modelica.SIunits.HeatFlowRate QCon_flow_internal(start=QCon_flow_nominal)=
            P - QEva_flow_internal "Condenser heat input";
          Modelica.SIunits.HeatFlowRate QEva_flow_internal(start=QEva_flow_nominal)=
            if COP_is_for_cooling then -COP * P else (1-COP)*P "Evaporator heat input";

          Modelica.Blocks.Sources.RealExpression yEva_flow_in(
            y=QEva_flow_internal/QEva_flow_nominal)
            "Normalized evaporator heat flow rate"
            annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
          Modelica.Blocks.Sources.RealExpression yCon_flow_in(
            y=QCon_flow_internal/QCon_flow_nominal)
            "Normalized condenser heat flow rate"
            annotation (Placement(transformation(extent={{-80,30},{-60,50}})));

          Modelica.Blocks.Math.Gain PEle(final k=P_nominal)
            "Electrical power consumption"
            annotation (Placement(transformation(extent={{60,-10},{80,10}})));
        equation

          connect(PEle.y, P)
            annotation (Line(points={{81,0},{110,0}}, color={0,0,127}));
          connect(PEle.u, y) annotation (Line(points={{58,0},{58,0},{40,0},{40,90},{-92,
                  90},{-120,90}},          color={0,0,127}));
          connect(yEva_flow_in.y, eva.u) annotation (Line(points={{-59,-40},{20,-40},{20,
                  -54},{12,-54}}, color={0,0,127}));
          connect(yCon_flow_in.y, con.u) annotation (Line(points={{-59,40},{-48,40},{-40,
                  40},{-40,66},{-12,66}}, color={0,0,127}));
          connect(con.Q_flow, QCon_flow) annotation (Line(points={{11,66},{20,66},{80,66},
                  {80,90},{110,90}}, color={0,0,127}));
          connect(eva.Q_flow, QEva_flow) annotation (Line(points={{-11,-54},{-20,-54},{-20,
                  -90},{110,-90}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}), graphics={
                Rectangle(
                  extent={{-56,68},{58,50}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-56,-52},{58,-70}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-103,64},{98,54}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,54},{98,64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-101,-56},{100,-66}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-100,-66},{0,-56}},
                  lineColor={0,0,127},
                  pattern=LinePattern.None,
                  fillColor={0,0,127},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,0},{-52,-12},{-32,-12},{-42,0}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,0},{-52,10},{-32,10},{-42,0}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,50},{-40,10}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,-12},{-40,-52}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{38,50},{42,-52}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{18,22},{62,-20}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{40,22},{22,-10},{58,-10},{40,22}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Text(
                  extent={{-130,128},{-78,106}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="y"),
                Text(extent={{66,28},{116,14}},   textString="P",
                  lineColor={0,0,127}),
                Line(points={{-100,90},{-80,90},{-80,14},{22,14}},
                                                            color={0,0,255}),
                Line(points={{62,0},{100,0}},                 color={0,0,255})}),
        defaultComponentName="chi",
        Documentation(info="<html>
<p>
This is a partial model of a chiller whose coefficient of performance (COP) changes
with temperatures in the same way as the Carnot efficiency changes.
This base class is used for the Carnot chiller and Carnot heat pump
that uses the leaving fluid temperature as the control signal.
</p>
</html>",
        revisions="<html>
<ul>
<li>
June 15, 2017, by Michael Wetter:<br/>
Added <code>min</code> attribute to parameter <code>P_nominal</code>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Implemented in the Annex 60 library the models
<a href=\"modelica://Buildings.Fluid.Chillers.Carnot_y\">Buildings.Fluid.Chillers.Carnot_y</a>
and
<a href=\"modelica://Buildings.Fluid.HeatPumps.Carnot_y\">Buildings.Fluid.HeatPumps.Carnot_y</a>
and refactored these models to use the same base class.<br/>
Implemented the removal of the flow direction dependency of
<code>staA1</code>, <code>staB1</code>, <code>staA2</code> and <code>staB2</code> as the
efficiency of the Carnot machine should only be computed in the design flow direction,
as corrected by Damien Picard.
</li>
<li>
December 18, 2015, by Michael Wetter:<br/>
Corrected wrong computation of <code>staB1</code> and <code>staB2</code>
which mistakenly used the <code>inStream</code> operator
for the configuration without flow reversal.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/476\">
issue 476</a>.
</li>
<li>
November 25, 2015 by Michael Wetter:<br/>
Changed sign convention for <code>dTEva_nominal</code> to be consistent with
other models.
The model will still work with the old values for <code>dTEva_nominal</code>,
but it will write a warning so that users can transition their models.
<br/>
Corrected <code>assert</code> statement for the efficiency curve.
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/468\">
issue 468</a>.
</li>
<li>
September 3, 2015 by Michael Wetter:<br/>
Expanded documentation.
</li>
<li>
May 6, 2015 by Michael Wetter:<br/>
Added <code>prescribedHeatFlowRate=true</code> for <code>vol2</code>.
</li>
<li>
October 9, 2013 by Michael Wetter:<br/>
Reimplemented the computation of the port states to avoid using
the conditionally removed variables <code>sta_a1</code>,
<code>sta_a2</code>, <code>sta_b1</code> and <code>sta_b2</code>.
</li>
<li>
May 10, 2013 by Michael Wetter:<br/>
Added electric power <code>P</code> as an output signal.
</li>
<li>
October 11, 2010 by Michael Wetter:<br/>
Fixed bug in energy balance.
</li>
<li>
March 3, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
        end PartialCarnot_y;

        partial model PartialElectric
          "Partial model for electric chiller based on the model in DOE-2, CoolTools and EnergyPlus"
          extends
            Task2_3.Task2c.Task2c_ed.BaseClassesChiller.FourPortHeatMassExchanger2(
            m1_flow_nominal=mCon_flow_nominal,
            m2_flow_nominal=mEva_flow_nominal,
            T1_start=273.15 + 25,
            T2_start=273.15 + 5,
            redeclare final Buildings.Fluid.MixingVolumes.MixingVolume vol2(
              V=m2_flow_nominal*tau2/rho2_nominal,
              nPorts=2,
              final prescribedHeatFlowRate=true),
            vol1(final prescribedHeatFlowRate=true));

          Modelica.Blocks.Interfaces.BooleanInput on
            "Set to true to enable compressor, or false to disable compressor"
            annotation (Placement(transformation(extent={{-140,10},{-100,50}}),
                iconTransformation(extent={{-140,10},{-100,50}})));
          Modelica.Blocks.Interfaces.RealInput TSet(unit="K", displayUnit="degC")
            "Set point for leaving chilled water temperature"
            annotation (Placement(transformation(extent={{-140,-50},{-100,-10}}),
                iconTransformation(extent={{-140,-50},{-100,-10}})));

          Modelica.SIunits.Temperature TEvaEnt "Evaporator entering temperature";
          Modelica.SIunits.Temperature TEvaLvg "Evaporator leaving temperature";
          Modelica.SIunits.Temperature TConEnt "Condenser entering temperature";
          Modelica.SIunits.Temperature TConLvg "Condenser leaving temperature";

          Modelica.SIunits.Efficiency COP "Coefficient of performance";
          Modelica.SIunits.HeatFlowRate QCon_flow "Condenser heat input";
          Modelica.SIunits.HeatFlowRate QEva_flow "Evaporator heat input";
          Modelica.Blocks.Interfaces.RealOutput P(final quantity="Power", unit="W")
            "Electric power consumed by compressor"
            annotation (Placement(transformation(extent={{100,80},{120,100}}),
                iconTransformation(extent={{100,80},{120,100}})));

          Real capFunT(min=0, nominal=1, start=1, unit="1")
            "Cooling capacity factor function of temperature curve";
          Modelica.SIunits.Efficiency EIRFunT(nominal=1, start=1)
            "Power input to cooling capacity ratio function of temperature curve";
          Modelica.SIunits.Efficiency EIRFunPLR(nominal=1, start=1)
            "Power input to cooling capacity ratio function of part load ratio";
          Real PLR1(min=0, nominal=1, start=1, unit="1") "Part load ratio";
          Real PLR2(min=0, nominal=1, start=1, unit="1") "Part load ratio";
          Real CR(min=0, nominal=1,  start=1, unit="1") "Cycling ratio";

        protected
          Modelica.SIunits.HeatFlowRate QEva_flow_ava(nominal=QEva_flow_nominal,start=QEva_flow_nominal)
            "Cooling capacity available at evaporator";
          Modelica.SIunits.HeatFlowRate QEva_flow_set(nominal=QEva_flow_nominal,start=QEva_flow_nominal)
            "Cooling capacity required to cool to set point temperature";
          Modelica.SIunits.SpecificEnthalpy hSet
            "Enthalpy setpoint for leaving chilled water";
          // Performance data
          parameter Modelica.SIunits.HeatFlowRate QEva_flow_nominal(max=0)
            "Reference capacity (negative number)";
          parameter Modelica.SIunits.Efficiency COP_nominal
            "Reference coefficient of performance";
          parameter Real PLRMax(min=0, unit="1") "Maximum part load ratio";
          parameter Real PLRMinUnl(min=0, unit="1") "Minimum part unload ratio";
          parameter Real PLRMin(min=0, unit="1") "Minimum part load ratio";
          parameter Modelica.SIunits.Efficiency etaMotor(max=1)
            "Fraction of compressor motor heat entering refrigerant";
          parameter Modelica.SIunits.MassFlowRate mEva_flow_nominal
            "Nominal mass flow at evaporator";
          parameter Modelica.SIunits.MassFlowRate mCon_flow_nominal
            "Nominal mass flow at condenser";
          parameter Modelica.SIunits.Temperature TEvaLvg_nominal
            "Temperature of fluid leaving evaporator at nominal condition";
          final parameter Modelica.SIunits.Conversions.NonSIunits.Temperature_degC
            TEvaLvg_nominal_degC=
            Modelica.SIunits.Conversions.to_degC(TEvaLvg_nominal)
            "Temperature of fluid leaving evaporator at nominal condition";
          Modelica.SIunits.Conversions.NonSIunits.Temperature_degC TEvaLvg_degC
            "Temperature of fluid leaving evaporator";
          parameter Modelica.SIunits.HeatFlowRate Q_flow_small = QEva_flow_nominal*1E-9
            "Small value for heat flow rate or power, used to avoid division by zero";
          Buildings.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloEva
            "Prescribed heat flow rate"
            annotation (Placement(transformation(extent={{-39,-50},{-19,-30}})));
          Buildings.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloCon
            "Prescribed heat flow rate"
            annotation (Placement(transformation(extent={{-37,30},{-17,50}})));
          Modelica.Blocks.Sources.RealExpression QEva_flow_in(y=QEva_flow)
            "Evaporator heat flow rate"
            annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
          Modelica.Blocks.Sources.RealExpression QCon_flow_in(y=QCon_flow)
            "Condenser heat flow rate"
            annotation (Placement(transformation(extent={{-80,30},{-60,50}})));

        initial equation
          assert(QEva_flow_nominal < 0, "Parameter QEva_flow_nominal must be smaller than zero.");
          assert(Q_flow_small < 0, "Parameter Q_flow_small must be smaller than zero.");
          assert(PLRMinUnl >= PLRMin, "Parameter PLRMinUnl must be bigger or equal to PLRMin");
          assert(PLRMax > PLRMinUnl, "Parameter PLRMax must be bigger than PLRMinUnl");
        equation
          // Condenser temperatures
          TConEnt = Medium1.temperature(Medium1.setState_phX(port_a1.p, inStream(port_a1.h_outflow)));
          TConLvg = vol1.heatPort.T;
          // Evaporator temperatures
          TEvaEnt = Medium2.temperature(Medium2.setState_phX(port_a2.p, inStream(port_a2.h_outflow)));
          TEvaLvg = vol2.heatPort.T;
          TEvaLvg_degC=Modelica.SIunits.Conversions.to_degC(TEvaLvg);

          // Enthalpy of temperature setpoint
          hSet = Medium2.specificEnthalpy_pTX(
                   p=port_b2.p,
                   T=TSet,
                   X=cat(1, port_b2.Xi_outflow, {1-sum(port_b2.Xi_outflow)}));

          if on then
            // Available cooling capacity
            QEva_flow_ava = QEva_flow_nominal*capFunT;
            // Cooling capacity required to chill water to setpoint
            QEva_flow_set = Buildings.Utilities.Math.Functions.smoothMin(
              x1 = m2_flow*(hSet-inStream(port_a2.h_outflow)),
              x2= Q_flow_small,
              deltaX=-Q_flow_small/100);

            // Part load ratio
            PLR1 = Buildings.Utilities.Math.Functions.smoothMin(
              x1 = QEva_flow_set/(QEva_flow_ava+Q_flow_small),
              x2 = PLRMax,
              deltaX=PLRMax/100);
            // PLR2 is the compressor part load ratio. The lower bound PLRMinUnl is
            // since for PLR1<PLRMinUnl, the chiller uses hot gas bypass, under which
            // condition the compressor power is assumed to be the same as if the chiller
            // were to operate at PLRMinUnl
            PLR2 = Buildings.Utilities.Math.Functions.smoothMax(
              x1 = PLRMinUnl,
              x2 = PLR1,
              deltaX = PLRMinUnl/100);

            // Cycling ratio.
            // Due to smoothing, this can be about deltaX/10 above 1.0
            CR = Buildings.Utilities.Math.Functions.smoothMin(
              x1 = PLR1/PLRMin,
              x2 = 1,
              deltaX=0.001);

            // Compressor power.
            P = -QEva_flow_ava/COP_nominal*EIRFunT*EIRFunPLR*CR;
            // Heat flow rates into evaporator and condenser
            // Q_flow_small is a negative number.
            QEva_flow = Buildings.Utilities.Math.Functions.smoothMax(
              x1 = QEva_flow_set,
              x2 = QEva_flow_ava,
              deltaX= -Q_flow_small/10);

          //QEva_flow = max(QEva_flow_set, QEva_flow_ava);
            QCon_flow = -QEva_flow + P*etaMotor;
            // Coefficient of performance
            COP = -QEva_flow/(P-Q_flow_small);
          else
            QEva_flow_ava = 0;
            QEva_flow_set = 0;
            PLR1 = 0;
            PLR2 = 0;
            CR   = 0;
            P    = 0;
            QEva_flow = 0;
            QCon_flow = 0;
            COP  = 0;
          end if;

          connect(QCon_flow_in.y, preHeaFloCon.Q_flow) annotation (Line(
              points={{-59,40},{-37,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(preHeaFloCon.port, vol1.heatPort) annotation (Line(
              points={{-17,40},{-10,40},{-10,60}},
              color={191,0,0},
              smooth=Smooth.None));
          connect(QEva_flow_in.y, preHeaFloEva.Q_flow) annotation (Line(
              points={{-59,-40},{-39,-40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(preHeaFloEva.port, vol2.heatPort) annotation (Line(
              points={{-19,-40},{12,-40},{12,-60}},
              color={191,0,0},
              smooth=Smooth.None));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}),
                           graphics={
                Text(extent={{62,96},{112,82}},   textString="P",
                  lineColor={0,0,127}),
                Text(extent={{-94,-24},{-48,-36}},  textString="T_CHWS",
                  lineColor={0,0,127}),
                Rectangle(
                  extent={{-99,-54},{102,-66}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-100,-66},{0,-54}},
                  lineColor={0,0,127},
                  pattern=LinePattern.None,
                  fillColor={0,0,127},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-104,66},{98,54}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,54},{98,66}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,52},{-40,12}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-56,70},{58,52}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,2},{-52,12},{-32,12},{-42,2}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-42,2},{-52,-10},{-32,-10},{-42,2}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-44,-10},{-40,-50}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{38,52},{42,-50}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-56,-50},{58,-68}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{18,24},{62,-18}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{40,24},{22,-8},{58,-8},{40,24}},
                  lineColor={0,0,0},
                  smooth=Smooth.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-108,36},{-62,24}},
                  lineColor={0,0,127},
                  textString="on")}),
        Documentation(info="<html>
<p>
Base class for model of an electric chiller, based on the DOE-2.1 chiller model and the
CoolTools chiller model that are implemented in EnergyPlus as the models
<code>Chiller:Electric:EIR</code> and <code>Chiller:Electric:ReformulatedEIR</code>.
</p>
<p>
The model takes as an input the set point for the leaving chilled water temperature,
which is met if the chiller has sufficient capacity.
Thus, the model has a built-in, ideal temperature control.
The model has three tests on the part load ratio and the cycling ratio:
</p>
<ol>
<li>
The test
<pre>
  PLR1 =min(QEva_flow_set/QEva_flow_ava, PLRMax);
</pre>
ensures that the chiller capacity does not exceed the chiller capacity specified
by the parameter <code>PLRMax</code>.
</li>
<li>
The test <pre>
  CR = min(PLR1/per.PRLMin, 1.0);
</pre>
computes a cycling ratio. This ratio expresses the fraction of time
that a chiller would run if it were to cycle because its load is smaller than
the minimal load at which it can operature. Notice that this model does continuously operature even if
the part load ratio is below the minimum part load ratio. Its leaving evaporator and condenser temperature
can therefore be considered as an
average temperature between the modes where the compressor is off and on.
</li>
<li>
The test <pre>
  PLR2 = max(PLRMinUnl, PLR1);
</pre>
computes the part load ratio of the compressor.
The assumption is that for a part load ratio below <code>PLRMinUnl</code>,
the chiller uses hot gas bypass to reduce the capacity, while the compressor
power draw does not change.
</li>
</ol>
<p>
The electric power only contains the power for the compressor, but not any power for pumps or fans.
</p>
<h4>Implementation</h4>
<p>
Models that extend from this base class need to provide
three functions to predict capacity and power consumption:
</p>
<ul>
<li>
A function to predict cooling capacity. The function value needs
to be assigned to <code>capFunT</code>.
</li>
<li>
A function to predict the power input as a function of temperature.
The function value needs to be assigned to <code>EIRFunT</code>.
</li>
<li>
A function to predict the power input as a function of the part load ratio.
The function value needs to be assigned to <code>EIRFunPLR</code>.
</li>
</ul>
</html>",
        revisions="<html>
<ul>
<li>
March 12, 2015, by Michael Wetter:<br/>
Refactored model to make it once continuously differentiable.
This is for issue <a href=\"https://github.com/lbl-srg/modelica-buildings/issues/373\">373</a>.
</li>
<li>
Jan. 10, 2011, by Michael Wetter:<br/>
Added input signal to switch chiller off, and changed base class to use a dynamic model.
The change of the base class was required to improve the robustness of the model when the control
is switched on again.
</li>
<li>
Sep. 8, 2010, by Michael Wetter:<br/>
Revised model and included it in the Buildings library.
</li>
<li>
October 13, 2008, by Brandon Hencey:<br/>
First implementation.
</li>
</ul>
</html>"));
        end PartialElectric;

        function warnIfPerformanceOutOfBounds
          "Function that checks the performance and writes a warning if it is outside of 0.9 to 1.1"
          input Real x "Argument to be checked";
          input String msg "String to be added to warning message";
          input String curveName "Name of the curve that was tested";
          output Integer retVal
            "0 if x is inside bounds, -1 if it is below bounds, or +1 if it is above bounds";

        algorithm
          if (x > 1.1) then
            retVal :=1;
          elseif ( x < 0.9) then
              retVal :=-1;
          else
            retVal :=0;
          end if;
          if (retVal <> 0) then
            Modelica.Utilities.Streams.print(
        "*** Warning: Chiller performance curves at nominal conditions are outside of bounds.
             "         + msg + " is outside of bounds 0.9 to 1.1.
             The value of the curve fit is "         + String(x) + "
             Check the coefficients of the function "         + curveName + ".");
          end if;

        annotation (
            Documentation(info="<html>
<p>
This function checks if the numeric argument is outside of the
interval <i>0.9</i> to <i>1.1</i>.
If this is the case, the function writes a warning.
</p>
</html>",         revisions="<html>
<ul>
<li>
September 12, 2010 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
        end warnIfPerformanceOutOfBounds;

        model FourPortHeatMassExchanger2
          "Model transporting two fluid streams between four ports with storing mass or energy"
          extends Buildings.Fluid.Interfaces.PartialFourPortInterface(
            port_a1(h_outflow(start=h1_outflow_start)),
            port_b1(h_outflow(start=h1_outflow_start)),
            port_a2(h_outflow(start=h2_outflow_start)),
            port_b2(h_outflow(start=h2_outflow_start)));
          extends Buildings.Fluid.Interfaces.FourPortFlowResistanceParameters(
             final computeFlowResistance1=true, final computeFlowResistance2=true);

          parameter Modelica.SIunits.Time tau1 = 30 "Time constant at nominal flow"
             annotation (Dialog(tab = "Dynamics", group="Nominal condition"));
          parameter Modelica.SIunits.Time tau2 = 30 "Time constant at nominal flow"
             annotation (Dialog(tab = "Dynamics", group="Nominal condition"));

          // Advanced
          parameter Boolean homotopyInitialization = true "= true, use homotopy method"
            annotation(Evaluate=true, Dialog(tab="Advanced"));

          // Assumptions
          parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
            "Type of energy balance: dynamic (3 initialization options) or steady state"
            annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
          parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
            "Type of mass balance: dynamic (3 initialization options) or steady state"
            annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

          // Initialization
          parameter Medium1.AbsolutePressure p1_start = Medium1.p_default
            "Start value of pressure"
            annotation(Dialog(tab = "Initialization", group = "Medium 1"));
          parameter Medium1.Temperature T1_start = Medium1.T_default
            "Start value of temperature"
            annotation(Dialog(tab = "Initialization", group = "Medium 1"));
          parameter Medium1.MassFraction X1_start[Medium1.nX] = Medium1.X_default
            "Start value of mass fractions m_i/m"
            annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nXi > 0));
          parameter Medium1.ExtraProperty C1_start[Medium1.nC](
            final quantity=Medium1.extraPropertiesNames)=fill(0, Medium1.nC)
            "Start value of trace substances"
            annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nC > 0));
          parameter Medium1.ExtraProperty C1_nominal[Medium1.nC](
            final quantity=Medium1.extraPropertiesNames) = fill(1E-2, Medium1.nC)
            "Nominal value of trace substances. (Set to typical order of magnitude.)"
           annotation (Dialog(tab="Initialization", group = "Medium 1", enable=Medium1.nC > 0));

          parameter Medium2.AbsolutePressure p2_start = Medium2.p_default
            "Start value of pressure"
            annotation(Dialog(tab = "Initialization", group = "Medium 2"));
          parameter Medium2.Temperature T2_start = Medium2.T_default
            "Start value of temperature"
            annotation(Dialog(tab = "Initialization", group = "Medium 2"));
          parameter Medium2.MassFraction X2_start[Medium2.nX] = Medium2.X_default
            "Start value of mass fractions m_i/m"
            annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nXi > 0));
          parameter Medium2.ExtraProperty C2_start[Medium2.nC](
            final quantity=Medium2.extraPropertiesNames)=fill(0, Medium2.nC)
            "Start value of trace substances"
            annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nC > 0));
          parameter Medium2.ExtraProperty C2_nominal[Medium2.nC](
            final quantity=Medium2.extraPropertiesNames) = fill(1E-2, Medium2.nC)
            "Nominal value of trace substances. (Set to typical order of magnitude.)"
           annotation (Dialog(tab="Initialization", group = "Medium 2", enable=Medium2.nC > 0));

          Modelica.SIunits.HeatFlowRate Q1_flow = vol1.heatPort.Q_flow
            "Heat flow rate into medium 1";
          Modelica.SIunits.HeatFlowRate Q2_flow = vol2.heatPort.Q_flow
            "Heat flow rate into medium 2";

          replaceable Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort vol1(nPorts=2)
            constrainedby
            Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort(
                redeclare final package Medium = Medium1,
                nPorts = 2,
                V=m1_flow_nominal*tau1/rho1_nominal,
                final allowFlowReversal=allowFlowReversal1,
                final m_flow_nominal=m1_flow_nominal,
                energyDynamics=if tau1 > Modelica.Constants.eps
                                 then energyDynamics else
                                 Modelica.Fluid.Types.Dynamics.SteadyState,
                massDynamics=if tau1 > Modelica.Constants.eps
                                 then massDynamics else
                                 Modelica.Fluid.Types.Dynamics.SteadyState,
                final p_start=p1_start,
                final T_start=T1_start,
                final X_start=X1_start,
                final C_start=C1_start,
                final C_nominal=C1_nominal,
                mSenFac=1) "Volume for fluid 1"
            annotation (Placement(transformation(extent={{-10,70}, {10,50}})));

          replaceable Buildings.Fluid.MixingVolumes.MixingVolume vol2
            constrainedby
            Buildings.Fluid.MixingVolumes.BaseClasses.MixingVolumeHeatPort(
                redeclare final package Medium = Medium2,
                nPorts = 2,
                V=m2_flow_nominal*tau2/rho2_nominal,
                final allowFlowReversal=allowFlowReversal2,
                mSenFac=1,
                final m_flow_nominal = m2_flow_nominal,
                energyDynamics=if tau2 > Modelica.Constants.eps
                                 then energyDynamics else
                                 Modelica.Fluid.Types.Dynamics.SteadyState,
                massDynamics=if tau2 > Modelica.Constants.eps
                                 then massDynamics else
                                 Modelica.Fluid.Types.Dynamics.SteadyState,
                final p_start=p2_start,
                final T_start=T2_start,
                final X_start=X2_start,
                final C_start=C2_start,
                final C_nominal=C2_nominal) "Volume for fluid 2"
           annotation (Placement(transformation(
                origin={2,-60},
                extent={{-10,10},{10,-10}},
                rotation=180)));

          Buildings.Fluid.FixedResistances.PressureDrop preDro1(
            redeclare final package Medium = Medium1,
            final m_flow_nominal=m1_flow_nominal,
            final deltaM=deltaM1,
            final allowFlowReversal=allowFlowReversal1,
            final show_T=false,
            final from_dp=from_dp1,
            final linearized=linearizeFlowResistance1,
            final homotopyInitialization=homotopyInitialization,
            final dp_nominal=dp1_nominal) "Flow resistance of fluid 1"
            annotation (Placement(transformation(extent={{-76,68},{-56,88}})));

          Buildings.Fluid.FixedResistances.PressureDrop preDro2(
            redeclare final package Medium = Medium2,
            final m_flow_nominal=m2_flow_nominal,
            final deltaM=deltaM2,
            final allowFlowReversal=allowFlowReversal2,
            final show_T=false,
            final from_dp=from_dp2,
            final linearized=linearizeFlowResistance2,
            final homotopyInitialization=homotopyInitialization,
            final dp_nominal=dp2_nominal) "Flow resistance of fluid 2"
            annotation (Placement(transformation(extent={{80,-90},{60,-70}})));

          Buildings.Fluid.Sensors.TemperatureTwoPort TAirConInt(redeclare
              package Medium =
                       Medium1, m_flow_nominal=m1_flow_nominal)
            "Temperature of air into condensor" annotation (Placement(transformation(
                extent={{-10,10},{10,-10}},
                rotation=0,
                origin={-32,78})));
        protected
          parameter Medium1.ThermodynamicState sta1_nominal=Medium1.setState_pTX(
              T=Medium1.T_default, p=Medium1.p_default, X=Medium1.X_default);
          parameter Modelica.SIunits.Density rho1_nominal=Medium1.density(sta1_nominal)
            "Density, used to compute fluid volume";
          parameter Medium2.ThermodynamicState sta2_nominal=Medium2.setState_pTX(
              T=Medium2.T_default, p=Medium2.p_default, X=Medium2.X_default);
          parameter Modelica.SIunits.Density rho2_nominal=Medium2.density(sta2_nominal)
            "Density, used to compute fluid volume";

          parameter Medium1.ThermodynamicState sta1_start=Medium1.setState_pTX(
              T=T1_start, p=p1_start, X=X1_start);
          parameter Modelica.SIunits.SpecificEnthalpy h1_outflow_start = Medium1.specificEnthalpy(sta1_start)
            "Start value for outflowing enthalpy";
          parameter Medium2.ThermodynamicState sta2_start=Medium2.setState_pTX(
              T=T2_start, p=p2_start, X=X2_start);
          parameter Modelica.SIunits.SpecificEnthalpy h2_outflow_start = Medium2.specificEnthalpy(sta2_start)
            "Start value for outflowing enthalpy";

        initial equation
          // Check for tau1
          assert((energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                  tau1 > Modelica.Constants.eps,
        "The parameter tau1, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau1 = "         + String(tau1) + "\n");
          assert((massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                  tau1 > Modelica.Constants.eps,
        "The parameter tau1, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau1 = "         + String(tau1) + "\n");

         // Check for tau2
          assert((energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                  tau2 > Modelica.Constants.eps,
        "The parameter tau2, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau2 = "         + String(tau2) + "\n");
          assert((massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) or
                  tau2 > Modelica.Constants.eps,
        "The parameter tau2, or the volume of the model from which tau may be derived, is unreasonably small.
 You need to set massDynamics == Modelica.Fluid.Types.Dynamics.SteadyState to model steady-state.
 Received tau2 = "         + String(tau2) + "\n");

        equation
          connect(vol1.ports[1], port_b1) annotation (Line(
              points={{-2,70},{20,70},{20,60},{100,60}},
              color={0,127,255}));
          connect(vol2.ports[2], port_b2) annotation (Line(
              points={{2,-70},{-30,-70},{-30,-60},{-100,-60}},
              color={0,127,255}));
          connect(port_a1, preDro1.port_a) annotation (Line(
              points={{-100,60},{-90,60},{-90,78},{-76,78}},
              color={0,127,255}));
          connect(port_a2, preDro2.port_a) annotation (Line(
              points={{100,-60},{90,-60},{90,-80},{80,-80}},
              color={0,127,255}));
          connect(preDro2.port_b, vol2.ports[1]) annotation (Line(
              points={{60,-80},{2,-80},{2,-70}},
              color={0,127,255}));
          connect(preDro1.port_b, TAirConInt.port_a)
            annotation (Line(points={{-56,78},{-42,78}}, color={0,127,255}));
          connect(TAirConInt.port_b, vol1.ports[2])
            annotation (Line(points={{-22,78},{2,78},{2,70}}, color={0,127,255}));
          annotation (
            Documentation(info="<html>
<p>
This component transports two fluid streams between four ports.
It provides the basic model for implementing a dynamic heat exchanger.
</p>
<p>
The model can be used as-is, although there will be no heat or mass transfer
between the two fluid streams.
To add heat transfer, heat flow can be added to the heat port of the two volumes.
See for example
<a href=\"Buildings.Fluid.Chillers.Carnot_y\">
Buildings.Fluid.Chillers.Carnot_y</a>.
To add moisture input into (or moisture output from) volume <code>vol2</code>,
the model can be replaced with
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolumeMoistAir\">
Buildings.Fluid.MixingVolumes.MixingVolumeMoistAir</a>.
</p>
<h4>Implementation</h4>
<p>
The variable names follow the conventions used in
<a href=\"modelica://Modelica.Fluid.Examples.HeatExchanger.BaseClasses.BasicHX\">
Modelica.Fluid.Examples.HeatExchanger.BaseClasses.BasicHX</a>.
</p>
</html>",         revisions="<html>
<ul>
<li>
October 23, 2017, by Michael Wetter:<br/>
Made volume <code>vol1</code> replaceable. This is required for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1013\">Buildings, issue 1013</a>.
</li>
<li>
December 1, 2016, by Michael Wetter:<br/>
Updated model as <code>use_dh</code> is no longer a parameter in the pressure drop model.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/480\">#480</a>.
</li>
<li>
April 11, 2016 by Michael Wetter:<br/>
Corrected wrong hyperlink in documentation for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/450\">issue 450</a>.
</li>
<li>
January 26, 2016, by Michael Wetter:<br/>
Set <code>quantity</code> attributes.
</li>
<li>
November 13, 2015, by Michael Wetter:<br/>
Changed assignments of start values in <code>extends</code> statement.
This is for issue
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/299\">#299</a>.
</li>
<li>
June 2, 2015, by Filip Jorissen:<br/>
Removed final modifier from <code>mSenFac</code> in
<code>vol1</code> and <code>vol2</code>.
This is for issue
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/258=\">#258</a>.
</li>
<li>
May 6, 2015, by Michael Wetter:<br/>
Added missing propagation of <code>allowFlowReversal</code> to
instances <code>vol1</code> and <code>vol2</code>.
This is for issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/412\">#412</a>.
</li>
<li>
October 6, 2014, by Michael Wetter:<br/>
Changed medium declaration in pressure drop elements to be final.
</li>
<li>
May 28, 2014, by Michael Wetter:<br/>
Removed <code>annotation(Evaluate=true)</code> for parameters <code>tau1</code>
and <code>tau2</code>.
This is needed to allow changing the time constant after translation.
</li>
<li>
November 12, 2013, by Michael Wetter:<br/>
Removed <code>import Modelica.Constants</code> statement.
</li>
<li>
October 8, 2013, by Michael Wetter:<br/>
Removed parameter <code>show_V_flow</code>.
</li>
<li>
September 26, 2013, by Michael Wetter:<br/>
Removed unrequired <code>sum</code> operator.
</li>
<li>
February 6, 2012, by Michael Wetter:<br/>
Updated documentation.
</li>
<li>
February 3, 2012, by Michael Wetter:<br/>
Removed assignment of <code>m_flow_small</code> as it is no
longer used in its base class.
</li>
<li>
July 29, 2011, by Michael Wetter:
<ul>
<li>
Changed values of
<code>h_outflow_a1_start</code>,
<code>h_outflow_b1_start</code>,
<code>h_outflow_a2_start</code> and
<code>h_outflow_b2_start</code>, and
declared them as final.
</li>
<li>
Set nominal values for <code>vol1.C</code> and <code>vol2.C</code>.
</li>
</ul>
</li>
<li>
July 11, 2011, by Michael Wetter:<br/>
Changed parameterization of fluid volume so that steady-state balance is
used when <code>tau = 0</code>.
</li>
<li>
March 25, 2011, by Michael Wetter:<br/>
Added homotopy operator.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
Added model to compute flow friction.
</li>
<li>
September 10, 2008 by Michael Wetter:<br/>
Added <code>stateSelect=StateSelect.always</code> for temperature of volume 1.
</li>
<li>
Changed temperature sensor from Celsius to Kelvin.
Unit conversion should be made during output
processing.
</li>
<li>
August 5, 2008, by Michael Wetter:<br/>
Replaced instances of <code>Delays.DelayFirstOrder</code> with instances of
<code>MixingVolumes.MixingVolume</code>. This allows to extract liquid for a condensing cooling
coil model.
</li>
<li>
March 25, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),  Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={1,1}), graphics={
                Rectangle(
                  extent={{-70,80},{70,-80}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={95,95,95},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-99,64},{102,54}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-99,-56},{102,-66}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid)}));
        end FourPortHeatMassExchanger2;
      annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://Buildings.Fluid.Chillers\">Buildings.Fluid.Chillers</a>.
</p>
</html>"));
      end BaseClassesChiller;
    end Task2c_ed;
  end Task2c;

  package Task3

    model PVSimpleOriented
      "Example for the PVSimpleOriented model with constant load"
      extends Modelica.Icons.Example;
      Buildings.Electrical.DC.Sources.PVSimpleOriented
                                                   pv(
        A=600,
        V_nominal=12,
        til=0.5235987755983,
        lat=0.43633231299858,
        azi=1.3089969389957)   "PV module"
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            origin={50,42})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-92,-40},{-72,-20}})));
      Buildings.Electrical.DC.Loads.Resistor    res(R=0.5, V_nominal=12)
        "Resistance"
        annotation (Placement(transformation(extent={{-2,-10},{18,10}})));
      Buildings.Electrical.DC.Sources.ConstantVoltage    sou(V=12) "Voltage source"
        annotation (Placement(transformation(extent={{-82,-10},{-62,10}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
          computeWetBulbTemperature=false, filNam=
            "C:/Users/BT/OneDrive - University College London/Engineered Environmental Elements/3E COURSEWORK/modelica-buildings-master/Task/ESP_Barcelona.081810_SWEC.mos")
        annotation (Placement(transformation(extent={{-128,90},{-108,110}})));
      Buildings.Electrical.DC.Lines.TwoPortResistance lin(R=0.05)
        "Transmission line"
        annotation (Placement(transformation(extent={{-38,30},{-18,50}})));
      Buildings.Electrical.DC.Sensors.GeneralizedSensor sen "Sensor"
        annotation (Placement(transformation(extent={{0,30},{20,50}})));
      Modelica.Blocks.Continuous.Integrator intPV
        annotation (Placement(transformation(extent={{10,12},{22,24}})));
    equation
      connect(sou.terminal, res.terminal) annotation (Line(
          points={{-62,0},{-2,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(lin.terminal_n, res.terminal) annotation (Line(
          points={{-38,40},{-50,40},{-50,0},{-2,0},{-2,5.55112e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(lin.terminal_p, sen.terminal_n) annotation (Line(
          points={{-18,40},{-4.44089e-16,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sen.terminal_p, pv.terminal) annotation (Line(
          points={{20,40},{30,40},{30,42},{40,42}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(sou.n, ground.p) annotation (Line(
          points={{-82,0},{-82,-20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(weaDat.weaBus, pv.weaBus) annotation (Line(
          points={{-108,100},{50,100},{50,51}},
          color={255,204,51},
          thickness=0.5,
          smooth=Smooth.None));
      connect(intPV.u, sen.P)
        annotation (Line(points={{8.8,18},{4,18},{4,31}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                -100},{100,140}})),
        experiment(
          StartTime=20044800,
          StopTime=20649600,
          Interval=3600,
          Tolerance=1e-06),
    Documentation(info="<html>
<p>
This model illustrates the use of the photovoltaic model.
The total solar irradiation is computed internally by the PV
model through a connection to the weather bus.
The PV is connected to a circuit that has a constant voltage
source and a resistance.
This voltage source may be a DC grid to which the
circuit is connected.
The power sensor shows how much electrical power is consumed or fed into the voltage source.
In actual systems, the voltage source may be an AC/DC converter.
</p>
</html>", revisions="<html>
<ul>
<li>
October 31, 2013, by Marco Bonvini:<br/>
First implementation.
</li>
</ul>
</html>"),
        __Dymola_Commands(file=
              "modelica://Buildings/Resources/Scripts/Dymola/Electrical/DC/Sources/Examples/PVSimpleOriented.mos"
            "Simulate and plot"),
        Icon(coordinateSystem(extent={{-140,-100},{100,140}})));
    end PVSimpleOriented;
  end Task3;
  annotation (uses(
      ElementsCWModel3(version="1"),
      Task2and3CW(version="1"),
      Modelica(version="3.2.3"),
      Buildings(version="6.0.0")),
    version="2",
    conversion(noneFromVersion="", noneFromVersion="1"));
end Task2_3;

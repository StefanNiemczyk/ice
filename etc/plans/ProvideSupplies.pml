<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1484927081537" name="ProvideSupplies" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states xsi:type="alica:SuccessState" id="1484927117903" name="Success" comment="">
    <inTransitions>#1484927389371</inTransitions>
  </states>
  <states id="1484927333656" name="FindSupplies" comment="" entryPoint="1484927081539">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/FindSupplies.beh#1484927591005</plans>
    <inTransitions>#1484927392769</inTransitions>
    <outTransitions>#1484927398698</outTransitions>
  </states>
  <states id="1484927341401" name="LoadSupplies" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/LoadSupplies.beh#1484927607502</plans>
    <inTransitions>#1484927398698</inTransitions>
    <outTransitions>#1484927424896</outTransitions>
  </states>
  <states id="1484927352148" name="FindVictim" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/FindVictim.beh#1484927617722</plans>
    <inTransitions>#1484927424896</inTransitions>
    <outTransitions>#1484927432879</outTransitions>
  </states>
  <states id="1484927371709" name="ProvideSupplies" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/ProvideSupplies.beh#1484927627341</plans>
    <inTransitions>#1484927432879</inTransitions>
    <outTransitions>#1484927389371</outTransitions>
    <outTransitions>#1484927392769</outTransitions>
  </states>
  <transitions id="1484927389371" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1484927391389" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484927371709</inState>
    <outState>#1484927117903</outState>
  </transitions>
  <transitions id="1484927392769" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1484927394940" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484927371709</inState>
    <outState>#1484927333656</outState>
  </transitions>
  <transitions id="1484927398698" name="toLoadSupplies" comment="toLoadSupplies" msg="toLoadSupplies">
    <preCondition id="1484927401848" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484927333656</inState>
    <outState>#1484927341401</outState>
  </transitions>
  <transitions id="1484927424896" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1484927426260" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484927341401</inState>
    <outState>#1484927352148</outState>
  </transitions>
  <transitions id="1484927432879" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1484927433844" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484927352148</inState>
    <outState>#1484927371709</outState>
  </transitions>
  <entryPoints id="1484927081539" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1414681164704</task>
    <state>#1484927333656</state>
  </entryPoints>
</alica:Plan>

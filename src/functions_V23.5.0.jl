# module NIDAQ

# using CEnum

const int8 = Int8

const uInt8 = Cuchar

const int16 = Cshort

const uInt16 = Cushort

const int32 = Cint

const uInt32 = Cuint

const float32 = Cfloat

const float64 = Cdouble

const int64 = Clonglong

const uInt64 = Culonglong

struct CVITime
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{CVITime}, f::Symbol)
    f === :lsb && return Ptr{uInt64}(x + 0)
    f === :msb && return Ptr{int64}(x + 8)
    return getfield(x, f)
end

function Base.getproperty(x::CVITime, f::Symbol)
    r = Ref{CVITime}(x)
    ptr = Base.unsafe_convert(Ptr{CVITime}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{CVITime}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct CVIAbsoluteTime
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{CVIAbsoluteTime}, f::Symbol)
    f === :cviTime && return Ptr{CVITime}(x + 0)
    f === :u32Data && return Ptr{NTuple{4, uInt32}}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::CVIAbsoluteTime, f::Symbol)
    r = Ref{CVIAbsoluteTime}(x)
    ptr = Base.unsafe_convert(Ptr{CVIAbsoluteTime}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{CVIAbsoluteTime}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const bool32 = uInt32

const TaskHandle = Ptr{Cvoid}

const CalHandle = uInt32

function DAQmxLoadTask(taskName, taskHandle)
    ccall((:DAQmxLoadTask, nidaqmx), int32, (Ptr{Cchar}, Ptr{TaskHandle}), taskName, taskHandle)
end

function DAQmxCreateTask(taskName, taskHandle)
    ccall((:DAQmxCreateTask, nidaqmx), int32, (Ptr{Cchar}, Ptr{TaskHandle}), taskName, taskHandle)
end

function DAQmxAddGlobalChansToTask(taskHandle, channelNames)
    ccall((:DAQmxAddGlobalChansToTask, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channelNames)
end

function DAQmxStartTask(taskHandle)
    ccall((:DAQmxStartTask, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxStopTask(taskHandle)
    ccall((:DAQmxStopTask, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxClearTask(taskHandle)
    ccall((:DAQmxClearTask, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxWaitUntilTaskDone(taskHandle, timeToWait)
    ccall((:DAQmxWaitUntilTaskDone, nidaqmx), int32, (TaskHandle, float64), taskHandle, timeToWait)
end

function DAQmxWaitForValidTimestamp(taskHandle, timestampEvent, timeout, timestamp)
    ccall((:DAQmxWaitForValidTimestamp, nidaqmx), int32, (TaskHandle, int32, float64, Ptr{CVIAbsoluteTime}), taskHandle, timestampEvent, timeout, timestamp)
end

function DAQmxIsTaskDone(taskHandle, isTaskDone)
    ccall((:DAQmxIsTaskDone, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, isTaskDone)
end

function DAQmxTaskControl(taskHandle, action)
    ccall((:DAQmxTaskControl, nidaqmx), int32, (TaskHandle, int32), taskHandle, action)
end

function DAQmxGetNthTaskChannel(taskHandle, index, buffer, bufferSize)
    ccall((:DAQmxGetNthTaskChannel, nidaqmx), int32, (TaskHandle, uInt32, Ptr{Cchar}, int32), taskHandle, index, buffer, bufferSize)
end

function DAQmxGetNthTaskDevice(taskHandle, index, buffer, bufferSize)
    ccall((:DAQmxGetNthTaskDevice, nidaqmx), int32, (TaskHandle, uInt32, Ptr{Cchar}, int32), taskHandle, index, buffer, bufferSize)
end

# typedef int32 ( CVICALLBACK * DAQmxEveryNSamplesEventCallbackPtr ) ( TaskHandle taskHandle , int32 everyNsamplesEventType , uInt32 nSamples , void * callbackData )
const DAQmxEveryNSamplesEventCallbackPtr = Ptr{Cvoid}

# typedef int32 ( CVICALLBACK * DAQmxDoneEventCallbackPtr ) ( TaskHandle taskHandle , int32 status , void * callbackData )
const DAQmxDoneEventCallbackPtr = Ptr{Cvoid}

# typedef int32 ( CVICALLBACK * DAQmxSignalEventCallbackPtr ) ( TaskHandle taskHandle , int32 signalID , void * callbackData )
const DAQmxSignalEventCallbackPtr = Ptr{Cvoid}

function DAQmxRegisterEveryNSamplesEvent(task, everyNsamplesEventType, nSamples, options, callbackFunction, callbackData)
    ccall((:DAQmxRegisterEveryNSamplesEvent, nidaqmx), int32, (TaskHandle, int32, uInt32, uInt32, DAQmxEveryNSamplesEventCallbackPtr, Ptr{Cvoid}), task, everyNsamplesEventType, nSamples, options, callbackFunction, callbackData)
end

function DAQmxRegisterDoneEvent(task, options, callbackFunction, callbackData)
    ccall((:DAQmxRegisterDoneEvent, nidaqmx), int32, (TaskHandle, uInt32, DAQmxDoneEventCallbackPtr, Ptr{Cvoid}), task, options, callbackFunction, callbackData)
end

function DAQmxRegisterSignalEvent(task, signalID, options, callbackFunction, callbackData)
    ccall((:DAQmxRegisterSignalEvent, nidaqmx), int32, (TaskHandle, int32, uInt32, DAQmxSignalEventCallbackPtr, Ptr{Cvoid}), task, signalID, options, callbackFunction, callbackData)
end

function DAQmxCreateAIVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateAIVoltageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateAICurrentChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
    ccall((:DAQmxCreateAICurrentChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
end

function DAQmxCreateAIVoltageRMSChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateAIVoltageRMSChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateAICurrentRMSChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
    ccall((:DAQmxCreateAICurrentRMSChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
end

function DAQmxCreateAIThrmcplChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, thermocoupleType, cjcSource, cjcVal, cjcChannel)
    ccall((:DAQmxCreateAIThrmcplChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, thermocoupleType, cjcSource, cjcVal, cjcChannel)
end

function DAQmxCreateAIRTDChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, rtdType, resistanceConfig, currentExcitSource, currentExcitVal, r0)
    ccall((:DAQmxCreateAIRTDChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, int32, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, rtdType, resistanceConfig, currentExcitSource, currentExcitVal, r0)
end

function DAQmxCreateAIThrmstrChanIex(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, a, b, c)
    ccall((:DAQmxCreateAIThrmstrChanIex, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, a, b, c)
end

function DAQmxCreateAIThrmstrChanVex(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, voltageExcitSource, voltageExcitVal, a, b, c, r1)
    ccall((:DAQmxCreateAIThrmstrChanVex, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, voltageExcitSource, voltageExcitVal, a, b, c, r1)
end

function DAQmxCreateAIFreqVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, thresholdLevel, hysteresis, customScaleName)
    ccall((:DAQmxCreateAIFreqVoltageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, float64, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, thresholdLevel, hysteresis, customScaleName)
end

function DAQmxCreateAIResistanceChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateAIResistanceChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateAIStrainGageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, strainConfig, voltageExcitSource, voltageExcitVal, gageFactor, initialBridgeVoltage, nominalGageResistance, poissonRatio, leadWireResistance, customScaleName)
    ccall((:DAQmxCreateAIStrainGageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64, float64, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, strainConfig, voltageExcitSource, voltageExcitVal, gageFactor, initialBridgeVoltage, nominalGageResistance, poissonRatio, leadWireResistance, customScaleName)
end

function DAQmxCreateAIRosetteStrainGageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, rosetteType, gageOrientation, rosetteMeasTypes, numRosetteMeasTypes, strainConfig, voltageExcitSource, voltageExcitVal, gageFactor, nominalGageResistance, poissonRatio, leadWireResistance)
    ccall((:DAQmxCreateAIRosetteStrainGageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, float64, Ptr{int32}, uInt32, int32, int32, float64, float64, float64, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, rosetteType, gageOrientation, rosetteMeasTypes, numRosetteMeasTypes, strainConfig, voltageExcitSource, voltageExcitVal, gageFactor, nominalGageResistance, poissonRatio, leadWireResistance)
end

function DAQmxCreateAIForceBridgeTwoPointLinChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIForceBridgeTwoPointLinChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
end

function DAQmxCreateAIForceBridgeTableChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIForceBridgeTableChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, int32, Ptr{float64}, uInt32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
end

function DAQmxCreateAIForceBridgePolynomialChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIForceBridgePolynomialChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, Ptr{float64}, uInt32, int32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
end

function DAQmxCreateAIPressureBridgeTwoPointLinChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIPressureBridgeTwoPointLinChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
end

function DAQmxCreateAIPressureBridgeTableChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIPressureBridgeTableChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, int32, Ptr{float64}, uInt32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
end

function DAQmxCreateAIPressureBridgePolynomialChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAIPressureBridgePolynomialChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, Ptr{float64}, uInt32, int32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
end

function DAQmxCreateAITorqueBridgeTwoPointLinChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAITorqueBridgeTwoPointLinChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, float64, float64, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, firstElectricalVal, secondElectricalVal, electricalUnits, firstPhysicalVal, secondPhysicalVal, physicalUnits, customScaleName)
end

function DAQmxCreateAITorqueBridgeTableChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAITorqueBridgeTableChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, int32, Ptr{float64}, uInt32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, electricalVals, numElectricalVals, electricalUnits, physicalVals, numPhysicalVals, physicalUnits, customScaleName)
end

function DAQmxCreateAITorqueBridgePolynomialChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
    ccall((:DAQmxCreateAITorqueBridgePolynomialChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{float64}, uInt32, Ptr{float64}, uInt32, int32, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, forwardCoeffs, numForwardCoeffs, reverseCoeffs, numReverseCoeffs, electricalUnits, physicalUnits, customScaleName)
end

function DAQmxCreateAIBridgeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, customScaleName)
    ccall((:DAQmxCreateAIBridgeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, nominalBridgeResistance, customScaleName)
end

function DAQmxCreateAIVoltageChanWithExcit(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, useExcitForScaling, customScaleName)
    ccall((:DAQmxCreateAIVoltageChanWithExcit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, int32, float64, bool32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, bridgeConfig, voltageExcitSource, voltageExcitVal, useExcitForScaling, customScaleName)
end

function DAQmxCreateAITempBuiltInSensorChan(taskHandle, physicalChannel, nameToAssignToChannel, units)
    ccall((:DAQmxCreateAITempBuiltInSensorChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32), taskHandle, physicalChannel, nameToAssignToChannel, units)
end

function DAQmxCreateAIAccelChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateAIAccelChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateAIVelocityIEPEChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateAIVelocityIEPEChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateAIForceIEPEChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateAIForceIEPEChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateAIMicrophoneChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, units, micSensitivity, maxSndPressLevel, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateAIMicrophoneChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, int32, float64, float64, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, units, micSensitivity, maxSndPressLevel, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateAIChargeChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateAIChargeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateAIAccelChargeChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, customScaleName)
    ccall((:DAQmxCreateAIAccelChargeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, customScaleName)
end

function DAQmxCreateAIAccel4WireDCVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, useExcitForScaling, customScaleName)
    ccall((:DAQmxCreateAIAccel4WireDCVoltageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, float64, int32, int32, float64, bool32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, useExcitForScaling, customScaleName)
end

function DAQmxCreateAIPosLVDTChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
    ccall((:DAQmxCreateAIPosLVDTChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, float64, int32, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
end

function DAQmxCreateAIPosRVDTChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
    ccall((:DAQmxCreateAIPosRVDTChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, float64, int32, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
end

function DAQmxCreateAIPosEddyCurrProxProbeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, customScaleName)
    ccall((:DAQmxCreateAIPosEddyCurrProxProbeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, sensitivity, sensitivityUnits, customScaleName)
end

function DAQmxCreateAIDeviceTempChan(taskHandle, physicalChannel, nameToAssignToChannel, units)
    ccall((:DAQmxCreateAIDeviceTempChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32), taskHandle, physicalChannel, nameToAssignToChannel, units)
end

function DAQmxCreateTEDSAIVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateTEDSAIVoltageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateTEDSAICurrentChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
    ccall((:DAQmxCreateTEDSAICurrentChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, shuntResistorLoc, extShuntResistorVal, customScaleName)
end

function DAQmxCreateTEDSAIThrmcplChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, cjcSource, cjcVal, cjcChannel)
    ccall((:DAQmxCreateTEDSAIThrmcplChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, cjcSource, cjcVal, cjcChannel)
end

function DAQmxCreateTEDSAIRTDChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal)
    ccall((:DAQmxCreateTEDSAIRTDChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal)
end

function DAQmxCreateTEDSAIThrmstrChanIex(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal)
    ccall((:DAQmxCreateTEDSAIThrmstrChanIex, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal)
end

function DAQmxCreateTEDSAIThrmstrChanVex(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, voltageExcitSource, voltageExcitVal, r1)
    ccall((:DAQmxCreateTEDSAIThrmstrChanVex, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, voltageExcitSource, voltageExcitVal, r1)
end

function DAQmxCreateTEDSAIResistanceChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIResistanceChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, resistanceConfig, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIStrainGageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, initialBridgeVoltage, leadWireResistance, customScaleName)
    ccall((:DAQmxCreateTEDSAIStrainGageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, float64, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, initialBridgeVoltage, leadWireResistance, customScaleName)
end

function DAQmxCreateTEDSAIForceBridgeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIForceBridgeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIPressureBridgeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIPressureBridgeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
end

function DAQmxCreateTEDSAITorqueBridgeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAITorqueBridgeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIBridgeChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIBridgeChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIVoltageChanWithExcit(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIVoltageChanWithExcit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIAccelChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIAccelChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIForceIEPEChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIForceIEPEChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, minVal, maxVal, units, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIMicrophoneChan(taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, units, maxSndPressLevel, currentExcitSource, currentExcitVal, customScaleName)
    ccall((:DAQmxCreateTEDSAIMicrophoneChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, int32, float64, int32, float64, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, terminalConfig, units, maxSndPressLevel, currentExcitSource, currentExcitVal, customScaleName)
end

function DAQmxCreateTEDSAIPosLVDTChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
    ccall((:DAQmxCreateTEDSAIPosLVDTChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
end

function DAQmxCreateTEDSAIPosRVDTChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
    ccall((:DAQmxCreateTEDSAIPosRVDTChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, voltageExcitSource, voltageExcitVal, voltageExcitFreq, ACExcitWireMode, customScaleName)
end

function DAQmxCreateAOVoltageChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateAOVoltageChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateAOCurrentChan(taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateAOCurrentChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, Ptr{Cchar}), taskHandle, physicalChannel, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateAOFuncGenChan(taskHandle, physicalChannel, nameToAssignToChannel, type, freq, amplitude, offset)
    ccall((:DAQmxCreateAOFuncGenChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, float64, float64, float64), taskHandle, physicalChannel, nameToAssignToChannel, type, freq, amplitude, offset)
end

function DAQmxCreateDIChan(taskHandle, lines, nameToAssignToLines, lineGrouping)
    ccall((:DAQmxCreateDIChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32), taskHandle, lines, nameToAssignToLines, lineGrouping)
end

function DAQmxCreateDOChan(taskHandle, lines, nameToAssignToLines, lineGrouping)
    ccall((:DAQmxCreateDOChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32), taskHandle, lines, nameToAssignToLines, lineGrouping)
end

function DAQmxCreateCIFreqChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, edge, measMethod, measTime, divisor, customScaleName)
    ccall((:DAQmxCreateCIFreqChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, uInt32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, edge, measMethod, measTime, divisor, customScaleName)
end

function DAQmxCreateCIPeriodChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, edge, measMethod, measTime, divisor, customScaleName)
    ccall((:DAQmxCreateCIPeriodChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, float64, uInt32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, edge, measMethod, measTime, divisor, customScaleName)
end

function DAQmxCreateCICountEdgesChan(taskHandle, counter, nameToAssignToChannel, edge, initialCount, countDirection)
    ccall((:DAQmxCreateCICountEdgesChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, uInt32, int32), taskHandle, counter, nameToAssignToChannel, edge, initialCount, countDirection)
end

function DAQmxCreateCIDutyCycleChan(taskHandle, counter, nameToAssignToChannel, minFreq, maxFreq, edge, customScaleName)
    ccall((:DAQmxCreateCIDutyCycleChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minFreq, maxFreq, edge, customScaleName)
end

function DAQmxCreateCIPulseWidthChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, startingEdge, customScaleName)
    ccall((:DAQmxCreateCIPulseWidthChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, startingEdge, customScaleName)
end

function DAQmxCreateCISemiPeriodChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
    ccall((:DAQmxCreateCISemiPeriodChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, customScaleName)
end

function DAQmxCreateCITwoEdgeSepChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, firstEdge, secondEdge, customScaleName)
    ccall((:DAQmxCreateCITwoEdgeSepChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, int32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units, firstEdge, secondEdge, customScaleName)
end

function DAQmxCreateCIPulseChanFreq(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units)
    ccall((:DAQmxCreateCIPulseChanFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units)
end

function DAQmxCreateCIPulseChanTime(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units)
    ccall((:DAQmxCreateCIPulseChanTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, units)
end

function DAQmxCreateCIPulseChanTicks(taskHandle, counter, nameToAssignToChannel, sourceTerminal, minVal, maxVal)
    ccall((:DAQmxCreateCIPulseChanTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, float64, float64), taskHandle, counter, nameToAssignToChannel, sourceTerminal, minVal, maxVal)
end

function DAQmxCreateCILinEncoderChan(taskHandle, counter, nameToAssignToChannel, decodingType, ZidxEnable, ZidxVal, ZidxPhase, units, distPerPulse, initialPos, customScaleName)
    ccall((:DAQmxCreateCILinEncoderChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, bool32, float64, int32, int32, float64, float64, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, decodingType, ZidxEnable, ZidxVal, ZidxPhase, units, distPerPulse, initialPos, customScaleName)
end

function DAQmxCreateCIAngEncoderChan(taskHandle, counter, nameToAssignToChannel, decodingType, ZidxEnable, ZidxVal, ZidxPhase, units, pulsesPerRev, initialAngle, customScaleName)
    ccall((:DAQmxCreateCIAngEncoderChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, bool32, float64, int32, int32, uInt32, float64, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, decodingType, ZidxEnable, ZidxVal, ZidxPhase, units, pulsesPerRev, initialAngle, customScaleName)
end

function DAQmxCreateCILinVelocityChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, decodingType, units, distPerPulse, customScaleName)
    ccall((:DAQmxCreateCILinVelocityChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, float64, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, decodingType, units, distPerPulse, customScaleName)
end

function DAQmxCreateCIAngVelocityChan(taskHandle, counter, nameToAssignToChannel, minVal, maxVal, decodingType, units, pulsesPerRev, customScaleName)
    ccall((:DAQmxCreateCIAngVelocityChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, int32, int32, uInt32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, minVal, maxVal, decodingType, units, pulsesPerRev, customScaleName)
end

function DAQmxCreateCIGPSTimestampChan(taskHandle, counter, nameToAssignToChannel, units, syncMethod, customScaleName)
    ccall((:DAQmxCreateCIGPSTimestampChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, int32, Ptr{Cchar}), taskHandle, counter, nameToAssignToChannel, units, syncMethod, customScaleName)
end

function DAQmxCreateCOPulseChanFreq(taskHandle, counter, nameToAssignToChannel, units, idleState, initialDelay, freq, dutyCycle)
    ccall((:DAQmxCreateCOPulseChanFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, int32, float64, float64, float64), taskHandle, counter, nameToAssignToChannel, units, idleState, initialDelay, freq, dutyCycle)
end

function DAQmxCreateCOPulseChanTime(taskHandle, counter, nameToAssignToChannel, units, idleState, initialDelay, lowTime, highTime)
    ccall((:DAQmxCreateCOPulseChanTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, int32, float64, float64, float64), taskHandle, counter, nameToAssignToChannel, units, idleState, initialDelay, lowTime, highTime)
end

function DAQmxCreateCOPulseChanTicks(taskHandle, counter, nameToAssignToChannel, sourceTerminal, idleState, initialDelay, lowTicks, highTicks)
    ccall((:DAQmxCreateCOPulseChanTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, int32, int32, int32, int32), taskHandle, counter, nameToAssignToChannel, sourceTerminal, idleState, initialDelay, lowTicks, highTicks)
end

function DAQmxCreateAIPowerChan(taskHandle, physicalChannel, nameToAssignToChannel, voltageSetpoint, currentSetpoint, outputEnable)
    ccall((:DAQmxCreateAIPowerChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, float64, float64, bool32), taskHandle, physicalChannel, nameToAssignToChannel, voltageSetpoint, currentSetpoint, outputEnable)
end

function DAQmxGetAIChanCalCalDate(taskHandle, channelName, year, month, day, hour, minute)
    ccall((:DAQmxGetAIChanCalCalDate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}), taskHandle, channelName, year, month, day, hour, minute)
end

function DAQmxSetAIChanCalCalDate(taskHandle, channelName, year, month, day, hour, minute)
    ccall((:DAQmxSetAIChanCalCalDate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32, uInt32, uInt32, uInt32, uInt32), taskHandle, channelName, year, month, day, hour, minute)
end

function DAQmxGetAIChanCalExpDate(taskHandle, channelName, year, month, day, hour, minute)
    ccall((:DAQmxGetAIChanCalExpDate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}), taskHandle, channelName, year, month, day, hour, minute)
end

function DAQmxSetAIChanCalExpDate(taskHandle, channelName, year, month, day, hour, minute)
    ccall((:DAQmxSetAIChanCalExpDate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32, uInt32, uInt32, uInt32, uInt32), taskHandle, channelName, year, month, day, hour, minute)
end

function DAQmxResetChanAttribute(taskHandle, channel, attribute)
    ccall((:DAQmxResetChanAttribute, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, attribute)
end

function DAQmxCfgSampClkTiming(taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan)
    ccall((:DAQmxCfgSampClkTiming, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, int32, uInt64), taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan)
end

function DAQmxCfgHandshakingTiming(taskHandle, sampleMode, sampsPerChan)
    ccall((:DAQmxCfgHandshakingTiming, nidaqmx), int32, (TaskHandle, int32, uInt64), taskHandle, sampleMode, sampsPerChan)
end

function DAQmxCfgBurstHandshakingTimingImportClock(taskHandle, sampleMode, sampsPerChan, sampleClkRate, sampleClkSrc, sampleClkActiveEdge, pauseWhen, readyEventActiveLevel)
    ccall((:DAQmxCfgBurstHandshakingTimingImportClock, nidaqmx), int32, (TaskHandle, int32, uInt64, float64, Ptr{Cchar}, int32, int32, int32), taskHandle, sampleMode, sampsPerChan, sampleClkRate, sampleClkSrc, sampleClkActiveEdge, pauseWhen, readyEventActiveLevel)
end

function DAQmxCfgBurstHandshakingTimingExportClock(taskHandle, sampleMode, sampsPerChan, sampleClkRate, sampleClkOutpTerm, sampleClkPulsePolarity, pauseWhen, readyEventActiveLevel)
    ccall((:DAQmxCfgBurstHandshakingTimingExportClock, nidaqmx), int32, (TaskHandle, int32, uInt64, float64, Ptr{Cchar}, int32, int32, int32), taskHandle, sampleMode, sampsPerChan, sampleClkRate, sampleClkOutpTerm, sampleClkPulsePolarity, pauseWhen, readyEventActiveLevel)
end

function DAQmxCfgChangeDetectionTiming(taskHandle, risingEdgeChan, fallingEdgeChan, sampleMode, sampsPerChan)
    ccall((:DAQmxCfgChangeDetectionTiming, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, uInt64), taskHandle, risingEdgeChan, fallingEdgeChan, sampleMode, sampsPerChan)
end

function DAQmxCfgImplicitTiming(taskHandle, sampleMode, sampsPerChan)
    ccall((:DAQmxCfgImplicitTiming, nidaqmx), int32, (TaskHandle, int32, uInt64), taskHandle, sampleMode, sampsPerChan)
end

function DAQmxCfgPipelinedSampClkTiming(taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan)
    ccall((:DAQmxCfgPipelinedSampClkTiming, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, int32, uInt64), taskHandle, source, rate, activeEdge, sampleMode, sampsPerChan)
end

function DAQmxResetTimingAttribute(taskHandle, attribute)
    ccall((:DAQmxResetTimingAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxResetTimingAttributeEx(taskHandle, deviceNames, attribute)
    ccall((:DAQmxResetTimingAttributeEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, deviceNames, attribute)
end

function DAQmxDisableStartTrig(taskHandle)
    ccall((:DAQmxDisableStartTrig, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxCfgDigEdgeStartTrig(taskHandle, triggerSource, triggerEdge)
    ccall((:DAQmxCfgDigEdgeStartTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, triggerSource, triggerEdge)
end

function DAQmxCfgAnlgEdgeStartTrig(taskHandle, triggerSource, triggerSlope, triggerLevel)
    ccall((:DAQmxCfgAnlgEdgeStartTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, float64), taskHandle, triggerSource, triggerSlope, triggerLevel)
end

function DAQmxCfgAnlgMultiEdgeStartTrig(taskHandle, triggerSources, triggerSlopeArray, triggerLevelArray, arraySize)
    ccall((:DAQmxCfgAnlgMultiEdgeStartTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}, Ptr{float64}, uInt32), taskHandle, triggerSources, triggerSlopeArray, triggerLevelArray, arraySize)
end

function DAQmxCfgAnlgWindowStartTrig(taskHandle, triggerSource, triggerWhen, windowTop, windowBottom)
    ccall((:DAQmxCfgAnlgWindowStartTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, float64, float64), taskHandle, triggerSource, triggerWhen, windowTop, windowBottom)
end

function DAQmxCfgTimeStartTrig(taskHandle, when, timescale)
    ccall((:DAQmxCfgTimeStartTrig, nidaqmx), int32, (TaskHandle, CVIAbsoluteTime, int32), taskHandle, when, timescale)
end

function DAQmxCfgDigPatternStartTrig(taskHandle, triggerSource, triggerPattern, triggerWhen)
    ccall((:DAQmxCfgDigPatternStartTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32), taskHandle, triggerSource, triggerPattern, triggerWhen)
end

function DAQmxDisableRefTrig(taskHandle)
    ccall((:DAQmxDisableRefTrig, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxCfgDigEdgeRefTrig(taskHandle, triggerSource, triggerEdge, pretriggerSamples)
    ccall((:DAQmxCfgDigEdgeRefTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, uInt32), taskHandle, triggerSource, triggerEdge, pretriggerSamples)
end

function DAQmxCfgAnlgEdgeRefTrig(taskHandle, triggerSource, triggerSlope, triggerLevel, pretriggerSamples)
    ccall((:DAQmxCfgAnlgEdgeRefTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, float64, uInt32), taskHandle, triggerSource, triggerSlope, triggerLevel, pretriggerSamples)
end

function DAQmxCfgAnlgMultiEdgeRefTrig(taskHandle, triggerSources, triggerSlopeArray, triggerLevelArray, pretriggerSamples, arraySize)
    ccall((:DAQmxCfgAnlgMultiEdgeRefTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}, Ptr{float64}, uInt32, uInt32), taskHandle, triggerSources, triggerSlopeArray, triggerLevelArray, pretriggerSamples, arraySize)
end

function DAQmxCfgAnlgWindowRefTrig(taskHandle, triggerSource, triggerWhen, windowTop, windowBottom, pretriggerSamples)
    ccall((:DAQmxCfgAnlgWindowRefTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, float64, float64, uInt32), taskHandle, triggerSource, triggerWhen, windowTop, windowBottom, pretriggerSamples)
end

function DAQmxCfgDigPatternRefTrig(taskHandle, triggerSource, triggerPattern, triggerWhen, pretriggerSamples)
    ccall((:DAQmxCfgDigPatternRefTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, int32, uInt32), taskHandle, triggerSource, triggerPattern, triggerWhen, pretriggerSamples)
end

function DAQmxResetTrigAttribute(taskHandle, attribute)
    ccall((:DAQmxResetTrigAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxReadAnalogF64(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadAnalogF64, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadAnalogScalarF64(taskHandle, timeout, value, reserved)
    ccall((:DAQmxReadAnalogScalarF64, nidaqmx), int32, (TaskHandle, float64, Ptr{float64}, Ptr{bool32}), taskHandle, timeout, value, reserved)
end

function DAQmxReadBinaryI16(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadBinaryI16, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{int16}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadBinaryU16(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadBinaryU16, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt16}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadBinaryI32(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadBinaryI32, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{int32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadBinaryU32(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadBinaryU32, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadDigitalU8(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadDigitalU8, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt8}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadDigitalU16(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadDigitalU16, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt16}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadDigitalU32(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadDigitalU32, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadDigitalScalarU32(taskHandle, timeout, value, reserved)
    ccall((:DAQmxReadDigitalScalarU32, nidaqmx), int32, (TaskHandle, float64, Ptr{uInt32}, Ptr{bool32}), taskHandle, timeout, value, reserved)
end

function DAQmxReadDigitalLines(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInBytes, sampsPerChanRead, numBytesPerSamp, reserved)
    ccall((:DAQmxReadDigitalLines, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt8}, uInt32, Ptr{int32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInBytes, sampsPerChanRead, numBytesPerSamp, reserved)
end

function DAQmxReadCounterF64(taskHandle, numSampsPerChan, timeout, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCounterF64, nidaqmx), int32, (TaskHandle, int32, float64, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCounterU32(taskHandle, numSampsPerChan, timeout, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCounterU32, nidaqmx), int32, (TaskHandle, int32, float64, Ptr{uInt32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCounterF64Ex(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCounterF64Ex, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCounterU32Ex(taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCounterU32Ex, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArray, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCounterScalarF64(taskHandle, timeout, value, reserved)
    ccall((:DAQmxReadCounterScalarF64, nidaqmx), int32, (TaskHandle, float64, Ptr{float64}, Ptr{bool32}), taskHandle, timeout, value, reserved)
end

function DAQmxReadCounterScalarU32(taskHandle, timeout, value, reserved)
    ccall((:DAQmxReadCounterScalarU32, nidaqmx), int32, (TaskHandle, float64, Ptr{uInt32}, Ptr{bool32}), taskHandle, timeout, value, reserved)
end

function DAQmxReadPowerF64(taskHandle, numSampsPerChan, timeout, fillMode, readArrayVoltage, readArrayCurrent, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadPowerF64, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{float64}, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArrayVoltage, readArrayCurrent, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadPowerBinaryI16(taskHandle, numSampsPerChan, timeout, fillMode, readArrayVoltage, readArrayCurrent, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadPowerBinaryI16, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{int16}, Ptr{int16}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, fillMode, readArrayVoltage, readArrayCurrent, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadPowerScalarF64(taskHandle, timeout, voltage, current, reserved)
    ccall((:DAQmxReadPowerScalarF64, nidaqmx), int32, (TaskHandle, float64, Ptr{float64}, Ptr{float64}, Ptr{bool32}), taskHandle, timeout, voltage, current, reserved)
end

function DAQmxReadCtrFreq(taskHandle, numSampsPerChan, timeout, interleaved, readArrayFrequency, readArrayDutyCycle, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCtrFreq, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{float64}, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, interleaved, readArrayFrequency, readArrayDutyCycle, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCtrTime(taskHandle, numSampsPerChan, timeout, interleaved, readArrayHighTime, readArrayLowTime, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCtrTime, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{float64}, Ptr{float64}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, interleaved, readArrayHighTime, readArrayLowTime, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCtrTicks(taskHandle, numSampsPerChan, timeout, interleaved, readArrayHighTicks, readArrayLowTicks, arraySizeInSamps, sampsPerChanRead, reserved)
    ccall((:DAQmxReadCtrTicks, nidaqmx), int32, (TaskHandle, int32, float64, bool32, Ptr{uInt32}, Ptr{uInt32}, uInt32, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, interleaved, readArrayHighTicks, readArrayLowTicks, arraySizeInSamps, sampsPerChanRead, reserved)
end

function DAQmxReadCtrFreqScalar(taskHandle, timeout, frequency, dutyCycle, reserved)
    ccall((:DAQmxReadCtrFreqScalar, nidaqmx), int32, (TaskHandle, float64, Ptr{float64}, Ptr{float64}, Ptr{bool32}), taskHandle, timeout, frequency, dutyCycle, reserved)
end

function DAQmxReadCtrTimeScalar(taskHandle, timeout, highTime, lowTime, reserved)
    ccall((:DAQmxReadCtrTimeScalar, nidaqmx), int32, (TaskHandle, float64, Ptr{float64}, Ptr{float64}, Ptr{bool32}), taskHandle, timeout, highTime, lowTime, reserved)
end

function DAQmxReadCtrTicksScalar(taskHandle, timeout, highTicks, lowTicks, reserved)
    ccall((:DAQmxReadCtrTicksScalar, nidaqmx), int32, (TaskHandle, float64, Ptr{uInt32}, Ptr{uInt32}, Ptr{bool32}), taskHandle, timeout, highTicks, lowTicks, reserved)
end

function DAQmxReadRaw(taskHandle, numSampsPerChan, timeout, readArray, arraySizeInBytes, sampsRead, numBytesPerSamp, reserved)
    ccall((:DAQmxReadRaw, nidaqmx), int32, (TaskHandle, int32, float64, Ptr{Cvoid}, uInt32, Ptr{int32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, timeout, readArray, arraySizeInBytes, sampsRead, numBytesPerSamp, reserved)
end

function DAQmxGetNthTaskReadChannel(taskHandle, index, buffer, bufferSize)
    ccall((:DAQmxGetNthTaskReadChannel, nidaqmx), int32, (TaskHandle, uInt32, Ptr{Cchar}, int32), taskHandle, index, buffer, bufferSize)
end

function DAQmxResetReadAttribute(taskHandle, attribute)
    ccall((:DAQmxResetReadAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxConfigureLogging(taskHandle, filePath, loggingMode, groupName, operation)
    ccall((:DAQmxConfigureLogging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32, Ptr{Cchar}, int32), taskHandle, filePath, loggingMode, groupName, operation)
end

function DAQmxStartNewFile(taskHandle, filePath)
    ccall((:DAQmxStartNewFile, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, filePath)
end

function DAQmxWriteAnalogF64(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteAnalogF64, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{float64}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteAnalogScalarF64(taskHandle, autoStart, timeout, value, reserved)
    ccall((:DAQmxWriteAnalogScalarF64, nidaqmx), int32, (TaskHandle, bool32, float64, float64, Ptr{bool32}), taskHandle, autoStart, timeout, value, reserved)
end

function DAQmxWriteBinaryI16(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteBinaryI16, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{int16}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteBinaryU16(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteBinaryU16, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt16}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteBinaryI32(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteBinaryI32, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{int32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteBinaryU32(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteBinaryU32, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteDigitalU8(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteDigitalU8, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt8}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteDigitalU16(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteDigitalU16, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt16}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteDigitalU32(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteDigitalU32, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteDigitalScalarU32(taskHandle, autoStart, timeout, value, reserved)
    ccall((:DAQmxWriteDigitalScalarU32, nidaqmx), int32, (TaskHandle, bool32, float64, uInt32, Ptr{bool32}), taskHandle, autoStart, timeout, value, reserved)
end

function DAQmxWriteDigitalLines(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteDigitalLines, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt8}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxWriteCtrFreq(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, frequency, dutyCycle, numSampsPerChanWritten, reserved)
    ccall((:DAQmxWriteCtrFreq, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{float64}, Ptr{float64}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, frequency, dutyCycle, numSampsPerChanWritten, reserved)
end

function DAQmxWriteCtrFreqScalar(taskHandle, autoStart, timeout, frequency, dutyCycle, reserved)
    ccall((:DAQmxWriteCtrFreqScalar, nidaqmx), int32, (TaskHandle, bool32, float64, float64, float64, Ptr{bool32}), taskHandle, autoStart, timeout, frequency, dutyCycle, reserved)
end

function DAQmxWriteCtrTime(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, highTime, lowTime, numSampsPerChanWritten, reserved)
    ccall((:DAQmxWriteCtrTime, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{float64}, Ptr{float64}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, highTime, lowTime, numSampsPerChanWritten, reserved)
end

function DAQmxWriteCtrTimeScalar(taskHandle, autoStart, timeout, highTime, lowTime, reserved)
    ccall((:DAQmxWriteCtrTimeScalar, nidaqmx), int32, (TaskHandle, bool32, float64, float64, float64, Ptr{bool32}), taskHandle, autoStart, timeout, highTime, lowTime, reserved)
end

function DAQmxWriteCtrTicks(taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, highTicks, lowTicks, numSampsPerChanWritten, reserved)
    ccall((:DAQmxWriteCtrTicks, nidaqmx), int32, (TaskHandle, int32, bool32, float64, bool32, Ptr{uInt32}, Ptr{uInt32}, Ptr{int32}, Ptr{bool32}), taskHandle, numSampsPerChan, autoStart, timeout, dataLayout, highTicks, lowTicks, numSampsPerChanWritten, reserved)
end

function DAQmxWriteCtrTicksScalar(taskHandle, autoStart, timeout, highTicks, lowTicks, reserved)
    ccall((:DAQmxWriteCtrTicksScalar, nidaqmx), int32, (TaskHandle, bool32, float64, uInt32, uInt32, Ptr{bool32}), taskHandle, autoStart, timeout, highTicks, lowTicks, reserved)
end

function DAQmxWriteRaw(taskHandle, numSamps, autoStart, timeout, writeArray, sampsPerChanWritten, reserved)
    ccall((:DAQmxWriteRaw, nidaqmx), int32, (TaskHandle, int32, bool32, float64, Ptr{Cvoid}, Ptr{int32}, Ptr{bool32}), taskHandle, numSamps, autoStart, timeout, writeArray, sampsPerChanWritten, reserved)
end

function DAQmxResetWriteAttribute(taskHandle, attribute)
    ccall((:DAQmxResetWriteAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxExportSignal(taskHandle, signalID, outputTerminal)
    ccall((:DAQmxExportSignal, nidaqmx), int32, (TaskHandle, int32, Ptr{Cchar}), taskHandle, signalID, outputTerminal)
end

function DAQmxResetExportedSignalAttribute(taskHandle, attribute)
    ccall((:DAQmxResetExportedSignalAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxCreateLinScale(name, slope, yIntercept, preScaledUnits, scaledUnits)
    ccall((:DAQmxCreateLinScale, nidaqmx), int32, (Ptr{Cchar}, float64, float64, int32, Ptr{Cchar}), name, slope, yIntercept, preScaledUnits, scaledUnits)
end

function DAQmxCreateMapScale(name, prescaledMin, prescaledMax, scaledMin, scaledMax, preScaledUnits, scaledUnits)
    ccall((:DAQmxCreateMapScale, nidaqmx), int32, (Ptr{Cchar}, float64, float64, float64, float64, int32, Ptr{Cchar}), name, prescaledMin, prescaledMax, scaledMin, scaledMax, preScaledUnits, scaledUnits)
end

function DAQmxCreatePolynomialScale(name, forwardCoeffs, numForwardCoeffsIn, reverseCoeffs, numReverseCoeffsIn, preScaledUnits, scaledUnits)
    ccall((:DAQmxCreatePolynomialScale, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32, Ptr{float64}, uInt32, int32, Ptr{Cchar}), name, forwardCoeffs, numForwardCoeffsIn, reverseCoeffs, numReverseCoeffsIn, preScaledUnits, scaledUnits)
end

function DAQmxCreateTableScale(name, prescaledVals, numPrescaledValsIn, scaledVals, numScaledValsIn, preScaledUnits, scaledUnits)
    ccall((:DAQmxCreateTableScale, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32, Ptr{float64}, uInt32, int32, Ptr{Cchar}), name, prescaledVals, numPrescaledValsIn, scaledVals, numScaledValsIn, preScaledUnits, scaledUnits)
end

function DAQmxCalculateReversePolyCoeff(forwardCoeffs, numForwardCoeffsIn, minValX, maxValX, numPointsToCompute, reversePolyOrder, reverseCoeffs)
    ccall((:DAQmxCalculateReversePolyCoeff, nidaqmx), int32, (Ptr{float64}, uInt32, float64, float64, int32, int32, Ptr{float64}), forwardCoeffs, numForwardCoeffsIn, minValX, maxValX, numPointsToCompute, reversePolyOrder, reverseCoeffs)
end

function DAQmxCfgInputBuffer(taskHandle, numSampsPerChan)
    ccall((:DAQmxCfgInputBuffer, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, numSampsPerChan)
end

function DAQmxCfgOutputBuffer(taskHandle, numSampsPerChan)
    ccall((:DAQmxCfgOutputBuffer, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, numSampsPerChan)
end

function DAQmxGetBufferAttribute(taskHandle, attribute, value)
    ccall((:DAQmxGetBufferAttribute, nidaqmx), int32, (TaskHandle, int32, Ptr{Cvoid}), taskHandle, attribute, value)
end

function DAQmxResetBufferAttribute(taskHandle, attribute)
    ccall((:DAQmxResetBufferAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxSwitchCreateScanList(scanList, taskHandle)
    ccall((:DAQmxSwitchCreateScanList, nidaqmx), int32, (Ptr{Cchar}, Ptr{TaskHandle}), scanList, taskHandle)
end

function DAQmxSwitchConnect(switchChannel1, switchChannel2, waitForSettling)
    ccall((:DAQmxSwitchConnect, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, bool32), switchChannel1, switchChannel2, waitForSettling)
end

function DAQmxSwitchConnectMulti(connectionList, waitForSettling)
    ccall((:DAQmxSwitchConnectMulti, nidaqmx), int32, (Ptr{Cchar}, bool32), connectionList, waitForSettling)
end

function DAQmxSwitchDisconnect(switchChannel1, switchChannel2, waitForSettling)
    ccall((:DAQmxSwitchDisconnect, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, bool32), switchChannel1, switchChannel2, waitForSettling)
end

function DAQmxSwitchDisconnectMulti(connectionList, waitForSettling)
    ccall((:DAQmxSwitchDisconnectMulti, nidaqmx), int32, (Ptr{Cchar}, bool32), connectionList, waitForSettling)
end

function DAQmxSwitchDisconnectAll(deviceName, waitForSettling)
    ccall((:DAQmxSwitchDisconnectAll, nidaqmx), int32, (Ptr{Cchar}, bool32), deviceName, waitForSettling)
end

function DAQmxSwitchSetTopologyAndReset(deviceName, newTopology)
    ccall((:DAQmxSwitchSetTopologyAndReset, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), deviceName, newTopology)
end

function DAQmxSwitchFindPath(switchChannel1, switchChannel2, path, pathBufferSize, pathStatus)
    ccall((:DAQmxSwitchFindPath, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, uInt32, Ptr{int32}), switchChannel1, switchChannel2, path, pathBufferSize, pathStatus)
end

function DAQmxSwitchOpenRelays(relayList, waitForSettling)
    ccall((:DAQmxSwitchOpenRelays, nidaqmx), int32, (Ptr{Cchar}, bool32), relayList, waitForSettling)
end

function DAQmxSwitchCloseRelays(relayList, waitForSettling)
    ccall((:DAQmxSwitchCloseRelays, nidaqmx), int32, (Ptr{Cchar}, bool32), relayList, waitForSettling)
end

function DAQmxSwitchGetSingleRelayCount(relayName, count)
    ccall((:DAQmxSwitchGetSingleRelayCount, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), relayName, count)
end

function DAQmxSwitchGetMultiRelayCount(relayList, count, countArraySize, numRelayCountsRead)
    ccall((:DAQmxSwitchGetMultiRelayCount, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, uInt32, Ptr{uInt32}), relayList, count, countArraySize, numRelayCountsRead)
end

function DAQmxSwitchGetSingleRelayPos(relayName, relayPos)
    ccall((:DAQmxSwitchGetSingleRelayPos, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), relayName, relayPos)
end

function DAQmxSwitchGetMultiRelayPos(relayList, relayPos, relayPosArraySize, numRelayPossRead)
    ccall((:DAQmxSwitchGetMultiRelayPos, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, uInt32, Ptr{uInt32}), relayList, relayPos, relayPosArraySize, numRelayPossRead)
end

function DAQmxSwitchWaitForSettling(deviceName)
    ccall((:DAQmxSwitchWaitForSettling, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxGetSwitchChanAttribute(switchChannelName, attribute, value)
    ccall((:DAQmxGetSwitchChanAttribute, nidaqmx), int32, (Ptr{Cchar}, int32, Ptr{Cvoid}), switchChannelName, attribute, value)
end

function DAQmxGetSwitchScanAttribute(taskHandle, attribute, value)
    ccall((:DAQmxGetSwitchScanAttribute, nidaqmx), int32, (TaskHandle, int32, Ptr{Cvoid}), taskHandle, attribute, value)
end

function DAQmxResetSwitchScanAttribute(taskHandle, attribute)
    ccall((:DAQmxResetSwitchScanAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxDisableAdvTrig(taskHandle)
    ccall((:DAQmxDisableAdvTrig, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxCfgDigEdgeAdvTrig(taskHandle, triggerSource, triggerEdge)
    ccall((:DAQmxCfgDigEdgeAdvTrig, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, triggerSource, triggerEdge)
end

function DAQmxSendSoftwareTrigger(taskHandle, triggerID)
    ccall((:DAQmxSendSoftwareTrigger, nidaqmx), int32, (TaskHandle, int32), taskHandle, triggerID)
end

function DAQmxConnectTerms(sourceTerminal, destinationTerminal, signalModifiers)
    ccall((:DAQmxConnectTerms, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, int32), sourceTerminal, destinationTerminal, signalModifiers)
end

function DAQmxDisconnectTerms(sourceTerminal, destinationTerminal)
    ccall((:DAQmxDisconnectTerms, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), sourceTerminal, destinationTerminal)
end

function DAQmxTristateOutputTerm(outputTerminal)
    ccall((:DAQmxTristateOutputTerm, nidaqmx), int32, (Ptr{Cchar},), outputTerminal)
end

function DAQmxResetDevice(deviceName)
    ccall((:DAQmxResetDevice, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxSelfTestDevice(deviceName)
    ccall((:DAQmxSelfTestDevice, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxCreateWatchdogTimerTaskEx(deviceName, taskName, taskHandle, timeout)
    ccall((:DAQmxCreateWatchdogTimerTaskEx, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{TaskHandle}, float64), deviceName, taskName, taskHandle, timeout)
end

function DAQmxControlWatchdogTask(taskHandle, action)
    ccall((:DAQmxControlWatchdogTask, nidaqmx), int32, (TaskHandle, int32), taskHandle, action)
end

function DAQmxCfgWatchdogAOExpirStates(taskHandle, channelNames, expirStateArray, outputTypeArray, arraySize)
    ccall((:DAQmxCfgWatchdogAOExpirStates, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, Ptr{int32}, uInt32), taskHandle, channelNames, expirStateArray, outputTypeArray, arraySize)
end

function DAQmxCfgWatchdogCOExpirStates(taskHandle, channelNames, expirStateArray, arraySize)
    ccall((:DAQmxCfgWatchdogCOExpirStates, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}, uInt32), taskHandle, channelNames, expirStateArray, arraySize)
end

function DAQmxCfgWatchdogDOExpirStates(taskHandle, channelNames, expirStateArray, arraySize)
    ccall((:DAQmxCfgWatchdogDOExpirStates, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}, uInt32), taskHandle, channelNames, expirStateArray, arraySize)
end

function DAQmxResetWatchdogAttribute(taskHandle, lines, attribute)
    ccall((:DAQmxResetWatchdogAttribute, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, lines, attribute)
end

function DAQmxSelfCal(deviceName)
    ccall((:DAQmxSelfCal, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxPerformBridgeOffsetNullingCal(taskHandle, channel)
    ccall((:DAQmxPerformBridgeOffsetNullingCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxPerformBridgeOffsetNullingCalEx(taskHandle, channel, skipUnsupportedChannels)
    ccall((:DAQmxPerformBridgeOffsetNullingCalEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, skipUnsupportedChannels)
end

function DAQmxPerformThrmcplLeadOffsetNullingCal(taskHandle, channel, skipUnsupportedChannels)
    ccall((:DAQmxPerformThrmcplLeadOffsetNullingCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, skipUnsupportedChannels)
end

function DAQmxPerformStrainShuntCal(taskHandle, channel, shuntResistorValue, shuntResistorLocation, skipUnsupportedChannels)
    ccall((:DAQmxPerformStrainShuntCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, bool32), taskHandle, channel, shuntResistorValue, shuntResistorLocation, skipUnsupportedChannels)
end

function DAQmxPerformStrainShuntCalEx(taskHandle, channel, shuntResistorValue, shuntResistorLocation, shuntResistorSelect, shuntResistorSource, skipUnsupportedChannels)
    ccall((:DAQmxPerformStrainShuntCalEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, int32, int32, bool32), taskHandle, channel, shuntResistorValue, shuntResistorLocation, shuntResistorSelect, shuntResistorSource, skipUnsupportedChannels)
end

function DAQmxPerformBridgeShuntCal(taskHandle, channel, shuntResistorValue, shuntResistorLocation, bridgeResistance, skipUnsupportedChannels)
    ccall((:DAQmxPerformBridgeShuntCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, float64, bool32), taskHandle, channel, shuntResistorValue, shuntResistorLocation, bridgeResistance, skipUnsupportedChannels)
end

function DAQmxPerformBridgeShuntCalEx(taskHandle, channel, shuntResistorValue, shuntResistorLocation, shuntResistorSelect, shuntResistorSource, bridgeResistance, skipUnsupportedChannels)
    ccall((:DAQmxPerformBridgeShuntCalEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64, int32, int32, int32, float64, bool32), taskHandle, channel, shuntResistorValue, shuntResistorLocation, shuntResistorSelect, shuntResistorSource, bridgeResistance, skipUnsupportedChannels)
end

function DAQmxGetSelfCalLastDateAndTime(deviceName, year, month, day, hour, minute)
    ccall((:DAQmxGetSelfCalLastDateAndTime, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}), deviceName, year, month, day, hour, minute)
end

function DAQmxGetExtCalLastDateAndTime(deviceName, year, month, day, hour, minute)
    ccall((:DAQmxGetExtCalLastDateAndTime, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}, Ptr{uInt32}), deviceName, year, month, day, hour, minute)
end

function DAQmxRestoreLastExtCalConst(deviceName)
    ccall((:DAQmxRestoreLastExtCalConst, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxESeriesCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxESeriesCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxMSeriesCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxMSeriesCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxSSeriesCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxSSeriesCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxSCBaseboardCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxSCBaseboardCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxAOSeriesCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxAOSeriesCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxXSeriesCalAdjust(calHandle, referenceVoltage)
    ccall((:DAQmxXSeriesCalAdjust, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxDeviceSupportsCal(deviceName, calSupported)
    ccall((:DAQmxDeviceSupportsCal, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), deviceName, calSupported)
end

function DAQmxInitExtCal(deviceName, password, calHandle)
    ccall((:DAQmxInitExtCal, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{CalHandle}), deviceName, password, calHandle)
end

function DAQmxCloseExtCal(calHandle, action)
    ccall((:DAQmxCloseExtCal, nidaqmx), int32, (CalHandle, int32), calHandle, action)
end

function DAQmxChangeExtCalPassword(deviceName, password, newPassword)
    ccall((:DAQmxChangeExtCalPassword, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}), deviceName, password, newPassword)
end

function DAQmxDSASetCalTemp(calHandle, temperature)
    ccall((:DAQmxDSASetCalTemp, nidaqmx), int32, (CalHandle, float64), calHandle, temperature)
end

function DAQmxAdjustDSAAICal(calHandle, referenceVoltage)
    ccall((:DAQmxAdjustDSAAICal, nidaqmx), int32, (CalHandle, float64), calHandle, referenceVoltage)
end

function DAQmxAdjustDSAAICalEx(calHandle, referenceVoltage, inputsShorted)
    ccall((:DAQmxAdjustDSAAICalEx, nidaqmx), int32, (CalHandle, float64, bool32), calHandle, referenceVoltage, inputsShorted)
end

function DAQmxAdjustDSAAICalWithGainAndCoupling(calHandle, coupling, gain, referenceVoltage)
    ccall((:DAQmxAdjustDSAAICalWithGainAndCoupling, nidaqmx), int32, (CalHandle, int32, float64, float64), calHandle, coupling, gain, referenceVoltage)
end

function DAQmxAdjustDSAAOCal(calHandle, channel, requestedLowVoltage, actualLowVoltage, requestedHighVoltage, actualHighVoltage, gainSetting)
    ccall((:DAQmxAdjustDSAAOCal, nidaqmx), int32, (CalHandle, uInt32, float64, float64, float64, float64, float64), calHandle, channel, requestedLowVoltage, actualLowVoltage, requestedHighVoltage, actualHighVoltage, gainSetting)
end

function DAQmxAdjust4610Cal(calHandle, channelName, gain, offset)
    ccall((:DAQmxAdjust4610Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelName, gain, offset)
end

function DAQmxAdjustDSATimebaseCal(calHandle, referenceFrequency)
    ccall((:DAQmxAdjustDSATimebaseCal, nidaqmx), int32, (CalHandle, float64), calHandle, referenceFrequency)
end

function DAQmxAdjustDSAAOTimebaseCal(calHandle, measuredFrequency, calComplete)
    ccall((:DAQmxAdjustDSAAOTimebaseCal, nidaqmx), int32, (CalHandle, float64, Ptr{bool32}), calHandle, measuredFrequency, calComplete)
end

function DAQmxSetupDSAAOTimebaseCal(calHandle, expectedFrequency)
    ccall((:DAQmxSetupDSAAOTimebaseCal, nidaqmx), int32, (CalHandle, Ptr{float64}), calHandle, expectedFrequency)
end

function DAQmxGet4463AdjustPoints(calHandle, terminalConfig, gain, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4463AdjustPoints, nidaqmx), int32, (CalHandle, int32, float64, Ptr{float64}, uInt32), calHandle, terminalConfig, gain, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4463Cal(calHandle, channelNames, referenceVoltage)
    ccall((:DAQmxAdjust4463Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, referenceVoltage)
end

function DAQmxSetup4463Cal(calHandle, channelNames, terminalConfig, gain, outputVoltage)
    ccall((:DAQmxSetup4463Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, float64, float64), calHandle, channelNames, terminalConfig, gain, outputVoltage)
end

function DAQmxSetup4480Cal(calHandle, channelNames, calMode)
    ccall((:DAQmxSetup4480Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, calMode)
end

function DAQmxAdjustTIOTimebaseCal(calHandle, referenceFrequency)
    ccall((:DAQmxAdjustTIOTimebaseCal, nidaqmx), int32, (CalHandle, float64), calHandle, referenceFrequency)
end

function DAQmxAdjust4204Cal(calHandle, channelNames, lowPassFreq, trackHoldEnabled, inputVal)
    ccall((:DAQmxAdjust4204Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, bool32, float64), calHandle, channelNames, lowPassFreq, trackHoldEnabled, inputVal)
end

function DAQmxAdjust4220Cal(calHandle, channelNames, gain, inputVal)
    ccall((:DAQmxAdjust4220Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, gain, inputVal)
end

function DAQmxAdjust4224Cal(calHandle, channelNames, gain, inputVal)
    ccall((:DAQmxAdjust4224Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, gain, inputVal)
end

function DAQmxAdjust4225Cal(calHandle, channelNames, gain, inputVal)
    ccall((:DAQmxAdjust4225Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, gain, inputVal)
end

function DAQmxSetup433xCal(calHandle, channelNames, excitationVoltage)
    ccall((:DAQmxSetup433xCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, excitationVoltage)
end

function DAQmxAdjust433xCal(calHandle, refVoltage, refExcitation, shuntLocation)
    ccall((:DAQmxAdjust433xCal, nidaqmx), int32, (CalHandle, float64, float64, int32), calHandle, refVoltage, refExcitation, shuntLocation)
end

function DAQmxSetup4339Cal(calHandle, channelNames, calMode, rangeMax, rangeMin, excitationVoltage)
    ccall((:DAQmxSetup4339Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, float64, float64, float64), calHandle, channelNames, calMode, rangeMax, rangeMin, excitationVoltage)
end

function DAQmxAdjust4339Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4339Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxGet4339CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4339CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4300Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4300Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxSetup4302Cal(calHandle, channelNames, rangeMin, rangeMax)
    ccall((:DAQmxSetup4302Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, rangeMin, rangeMax)
end

function DAQmxGet4302CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4302CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4302Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4302Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxSetup4303Cal(calHandle, channelNames, rangeMin, rangeMax)
    ccall((:DAQmxSetup4303Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, rangeMin, rangeMax)
end

function DAQmxGet4303CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4303CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4303Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4303Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxSetup4304Cal(calHandle, channelNames, rangeMin, rangeMax)
    ccall((:DAQmxSetup4304Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, rangeMin, rangeMax)
end

function DAQmxGet4304CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4304CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4304Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4304Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxSetup4305Cal(calHandle, channelNames, rangeMin, rangeMax)
    ccall((:DAQmxSetup4305Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, rangeMin, rangeMax)
end

function DAQmxGet4305CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4305CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust4305Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4305Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxAdjust4309Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4309Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxAdjust4310Cal(calHandle, refVoltage)
    ccall((:DAQmxAdjust4310Cal, nidaqmx), int32, (CalHandle, float64), calHandle, refVoltage)
end

function DAQmxAdjust4353Cal(calHandle, channelNames, refVal)
    ccall((:DAQmxAdjust4353Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, refVal)
end

function DAQmxAdjust4357Cal(calHandle, channelNames, refVals, numRefVals)
    ccall((:DAQmxAdjust4357Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, Ptr{float64}, int32), calHandle, channelNames, refVals, numRefVals)
end

function DAQmxSetup4322Cal(calHandle, channelNames, outputType, outputVal)
    ccall((:DAQmxSetup4322Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, float64), calHandle, channelNames, outputType, outputVal)
end

function DAQmxAdjust4322Cal(calHandle, channelNames, refVal)
    ccall((:DAQmxAdjust4322Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, refVal)
end

function DAQmxGet4322CalAdjustPoints(calHandle, outputType, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet4322CalAdjustPoints, nidaqmx), int32, (CalHandle, int32, Ptr{float64}, uInt32), calHandle, outputType, adjustmentPoints, bufferSize)
end

function DAQmxConnectSCExpressCalAccChans(calHandle, channelNames, connection)
    ccall((:DAQmxConnectSCExpressCalAccChans, nidaqmx), int32, (CalHandle, Ptr{Cchar}, Ptr{Cchar}), calHandle, channelNames, connection)
end

function DAQmxDisconnectSCExpressCalAccChans(calHandle)
    ccall((:DAQmxDisconnectSCExpressCalAccChans, nidaqmx), int32, (CalHandle,), calHandle)
end

function DAQmxGetPossibleSCExpressCalAccConnections(deviceName, channelNames, connections, connectionsBufferSize)
    ccall((:DAQmxGetPossibleSCExpressCalAccConnections, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, uInt32), deviceName, channelNames, connections, connectionsBufferSize)
end

function DAQmxSetSCExpressCalAccBridgeOutput(calHandle, voltsPerVolt)
    ccall((:DAQmxSetSCExpressCalAccBridgeOutput, nidaqmx), int32, (CalHandle, float64), calHandle, voltsPerVolt)
end

function DAQmxFieldDAQSetCalTemp(calHandle, temperature)
    ccall((:DAQmxFieldDAQSetCalTemp, nidaqmx), int32, (CalHandle, float64), calHandle, temperature)
end

function DAQmxGet11601CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11601CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11601Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11601Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet11603CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11603CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11603Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11603Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxSetup11605Cal(calHandle, rangeMin, rangeMax)
    ccall((:DAQmxSetup11605Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, rangeMin, rangeMax)
end

function DAQmxGet11605CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11605CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11605Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11605Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet11613CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11613CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11613Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11613Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet11614CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11614CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11614Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11614Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxSetup11634Cal(calHandle, rangeMin, rangeMax)
    ccall((:DAQmxSetup11634Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, rangeMin, rangeMax)
end

function DAQmxGet11634CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet11634CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust11634Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust11634Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxSetup11637Cal(calHandle, channelNames, bridgeConfig, voltageExcitation)
    ccall((:DAQmxSetup11637Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, float64), calHandle, channelNames, bridgeConfig, voltageExcitation)
end

function DAQmxAdjust11637Cal(calHandle, value, actualReading, asFoundGainError, asFoundOffsetError)
    ccall((:DAQmxAdjust11637Cal, nidaqmx), int32, (CalHandle, float64, Ptr{float64}, Ptr{float64}, Ptr{float64}), calHandle, value, actualReading, asFoundGainError, asFoundOffsetError)
end

function DAQmxGet9201CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9201CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxCSeriesSetCalTemp(calHandle, temperature)
    ccall((:DAQmxCSeriesSetCalTemp, nidaqmx), int32, (CalHandle, float64), calHandle, temperature)
end

function DAQmxAdjust9201Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9201Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9202CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9202CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9202Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9202Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9203CalAdjustPoints(calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9203CalAdjustPoints, nidaqmx), int32, (CalHandle, float64, float64, Ptr{float64}, uInt32), calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9203GainCal(calHandle, channelNames, rangeMin, rangeMax, value)
    ccall((:DAQmxAdjust9203GainCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, float64), calHandle, channelNames, rangeMin, rangeMax, value)
end

function DAQmxAdjust9203OffsetCal(calHandle, channelNames, rangeMin, rangeMax)
    ccall((:DAQmxAdjust9203OffsetCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelNames, rangeMin, rangeMax)
end

function DAQmxAdjust9205Cal(calHandle, value)
    ccall((:DAQmxAdjust9205Cal, nidaqmx), int32, (CalHandle, float64), calHandle, value)
end

function DAQmxAdjust9206Cal(calHandle, value)
    ccall((:DAQmxAdjust9206Cal, nidaqmx), int32, (CalHandle, float64), calHandle, value)
end

function DAQmxGet9207CalAdjustPoints(calHandle, channelNames, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9207CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{Cchar}, Ptr{float64}, uInt32), calHandle, channelNames, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9207GainCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9207GainCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9207OffsetCal(calHandle, channelNames)
    ccall((:DAQmxAdjust9207OffsetCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelNames)
end

function DAQmxGet9208CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9208CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9208GainCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9208GainCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9208OffsetCal(calHandle, channelNames)
    ccall((:DAQmxAdjust9208OffsetCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelNames)
end

function DAQmxGet9209CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9209CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9209GainCal(calHandle, channelNames, terminalConfig, value)
    ccall((:DAQmxAdjust9209GainCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, float64), calHandle, channelNames, terminalConfig, value)
end

function DAQmxAdjust9209OffsetCal(calHandle, channelNames)
    ccall((:DAQmxAdjust9209OffsetCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelNames)
end

function DAQmxAdjust9210Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9210Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9211Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9211Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9212CalAdjustPoints(calHandle, channelNames, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9212CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{Cchar}, Ptr{float64}, uInt32), calHandle, channelNames, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9212Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9212Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9213CalAdjustPoints(calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9213CalAdjustPoints, nidaqmx), int32, (CalHandle, float64, float64, Ptr{float64}, uInt32), calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9213Cal(calHandle, channelNames, rangeMin, rangeMax, value)
    ccall((:DAQmxAdjust9213Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, float64), calHandle, channelNames, rangeMin, rangeMax, value)
end

function DAQmxGet9214CalAdjustPoints(calHandle, channelNames, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9214CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{Cchar}, Ptr{float64}, uInt32), calHandle, channelNames, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9214Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9214Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9215CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9215CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9215Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9215Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9216CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9216CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9216Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9216Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9217CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9217CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9217Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9217Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxSetup9218Cal(calHandle, channelNames, rangeMin, rangeMax, measType)
    ccall((:DAQmxSetup9218Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, int32), calHandle, channelNames, rangeMin, rangeMax, measType)
end

function DAQmxGet9218CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9218CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9218Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9218Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxSetup9219Cal(calHandle, channelNames, rangeMin, rangeMax, measType, bridgeConfig)
    ccall((:DAQmxSetup9219Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, int32, int32), calHandle, channelNames, rangeMin, rangeMax, measType, bridgeConfig)
end

function DAQmxGet9219CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9219CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9219Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9219Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9220CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9220CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9220Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9220Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9221CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9221CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9221Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9221Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9222CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9222CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9222Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9222Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9223CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9223CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9223Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9223Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9224CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9224CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9224Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9224Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9225CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9225CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9225Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9225Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9226CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9226CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9226Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9226Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9227CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9227CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9227Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9227Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9228CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9228CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9228Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9228Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9229CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9229CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9229Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9229Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9230CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9230CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9230Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9230Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9231CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9231CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9231Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9231Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9232CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9232CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9232Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9232Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9234CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9234CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9234GainCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9234GainCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9234OffsetCal(calHandle, channelNames)
    ccall((:DAQmxAdjust9234OffsetCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelNames)
end

function DAQmxGet9238CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9238CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9238Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9238Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9239CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9239CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9239Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9239Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9242CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9242CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9242Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9242Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9242Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9242Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9244CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9244CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9244Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9244Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust9244Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9244Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9246CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9246CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9246Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9246Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9247CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9247CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9247Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9247Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9250CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9250CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9250Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9250Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9251CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9251CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9251Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9251Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9252CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9252CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9252Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9252Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9253CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9253CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{float64}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9253Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9253Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9260CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9260CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9260Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9260Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9260Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9260Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9262CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9262CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9262Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9262Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9262Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9262Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9263CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9263CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9263Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9263Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9263Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9263Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9264CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9264CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9264Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9264Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9264Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9264Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9265CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9265CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9265Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9265Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9265Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9265Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9266CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9266CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9266Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9266Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9266Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9266Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9269CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9269CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9269Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9269Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9269Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9269Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9628AICalAdjustPoints(calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9628AICalAdjustPoints, nidaqmx), int32, (CalHandle, float64, float64, Ptr{float64}, uInt32), calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9628AICal(calHandle, channelNames, rangeMin, rangeMax, value)
    ccall((:DAQmxAdjust9628AICal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, float64), calHandle, channelNames, rangeMin, rangeMax, value)
end

function DAQmxGet9629AICalAdjustPoints(calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9629AICalAdjustPoints, nidaqmx), int32, (CalHandle, float64, float64, Ptr{float64}, uInt32), calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9629AICal(calHandle, channelNames, rangeMin, rangeMax, value)
    ccall((:DAQmxAdjust9629AICal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, float64), calHandle, channelNames, rangeMin, rangeMax, value)
end

function DAQmxGet9638AICalAdjustPoints(calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9638AICalAdjustPoints, nidaqmx), int32, (CalHandle, float64, float64, Ptr{float64}, uInt32), calHandle, rangeMin, rangeMax, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9638AICal(calHandle, channelNames, rangeMin, rangeMax, value)
    ccall((:DAQmxAdjust9638AICal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64, float64), calHandle, channelNames, rangeMin, rangeMax, value)
end

function DAQmxGet9628AOCalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9628AOCalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9628AOCal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9628AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9628AOCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9628AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9629AOCalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9629AOCalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9629AOCal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9629AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9629AOCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9629AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9638AOCalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9638AOCalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup9638AOCal(calHandle, channelNames, value)
    ccall((:DAQmxSetup9638AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust9638AOCal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust9638AOCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxGet9775CalAdjustPoints(calHandle, coupling, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet9775CalAdjustPoints, nidaqmx), int32, (CalHandle, uInt32, Ptr{float64}, uInt32), calHandle, coupling, adjustmentPoints, bufferSize)
end

function DAQmxAdjust9775Cal(calHandle, channelNames, value, coupling)
    ccall((:DAQmxAdjust9775Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, uInt32), calHandle, channelNames, value, coupling)
end

function DAQmxGet15110CalAdjustPoints(calHandle, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet15110CalAdjustPoints, nidaqmx), int32, (CalHandle, Ptr{int32}, uInt32), calHandle, adjustmentPoints, bufferSize)
end

function DAQmxSetup15110Cal(calHandle, channelNames, value)
    ccall((:DAQmxSetup15110Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32), calHandle, channelNames, value)
end

function DAQmxAdjust15110Cal(calHandle, channelNames, value)
    ccall((:DAQmxAdjust15110Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelNames, value)
end

function DAQmxAdjust15100Cal(calHandle, value)
    ccall((:DAQmxAdjust15100Cal, nidaqmx), int32, (CalHandle, float64), calHandle, value)
end

function DAQmxGet15200CalAdjustPoints(calHandle, powerCalibrationType, adjustmentPoints, bufferSize)
    ccall((:DAQmxGet15200CalAdjustPoints, nidaqmx), int32, (CalHandle, int32, Ptr{float64}, uInt32), calHandle, powerCalibrationType, adjustmentPoints, bufferSize)
end

function DAQmxSetup15200Cal(calHandle, channelNames, adjustmentPoint, powerCalibrationType)
    ccall((:DAQmxSetup15200Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, int32), calHandle, channelNames, adjustmentPoint, powerCalibrationType)
end

function DAQmxAdjust15200Cal(calHandle, channelNames, referenceValue, powerCalibrationType)
    ccall((:DAQmxAdjust15200Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, int32), calHandle, channelNames, referenceValue, powerCalibrationType)
end

function DAQmxSetup1102Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1102Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1102Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1102Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1104Cal(calHandle, channelName)
    ccall((:DAQmxSetup1104Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelName)
end

function DAQmxAdjust1104Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1104Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1112Cal(calHandle, channelName)
    ccall((:DAQmxSetup1112Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelName)
end

function DAQmxAdjust1112Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1112Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1122Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1122Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1122Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1122Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1124Cal(calHandle, channelName, range, dacValue)
    ccall((:DAQmxSetup1124Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, int32, uInt32), calHandle, channelName, range, dacValue)
end

function DAQmxAdjust1124Cal(calHandle, measOutput)
    ccall((:DAQmxAdjust1124Cal, nidaqmx), int32, (CalHandle, float64), calHandle, measOutput)
end

function DAQmxSetup1125Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1125Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1125Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1125Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1126Cal(calHandle, channelName, upperFreqLimit)
    ccall((:DAQmxSetup1126Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, upperFreqLimit)
end

function DAQmxAdjust1126Cal(calHandle, refFreq, measOutput)
    ccall((:DAQmxAdjust1126Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refFreq, measOutput)
end

function DAQmxSetup1141Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1141Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1141Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1141Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1142Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1142Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1142Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1142Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1143Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1143Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1143Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1143Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1502Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1502Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1502Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1502Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1503Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1503Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1503Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1503Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxAdjust1503CurrentCal(calHandle, channelName, measCurrent)
    ccall((:DAQmxAdjust1503CurrentCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, measCurrent)
end

function DAQmxSetup1520Cal(calHandle, channelName, gain)
    ccall((:DAQmxSetup1520Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust1520Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1520Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1521Cal(calHandle, channelName)
    ccall((:DAQmxSetup1521Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}), calHandle, channelName)
end

function DAQmxAdjust1521Cal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust1521Cal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup153xCal(calHandle, channelName, gain)
    ccall((:DAQmxSetup153xCal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64), calHandle, channelName, gain)
end

function DAQmxAdjust153xCal(calHandle, refVoltage, measOutput)
    ccall((:DAQmxAdjust153xCal, nidaqmx), int32, (CalHandle, float64, float64), calHandle, refVoltage, measOutput)
end

function DAQmxSetup1540Cal(calHandle, channelName, excitationVoltage, excitationFreq)
    ccall((:DAQmxSetup1540Cal, nidaqmx), int32, (CalHandle, Ptr{Cchar}, float64, float64), calHandle, channelName, excitationVoltage, excitationFreq)
end

function DAQmxAdjust1540Cal(calHandle, refVoltage, measOutput, inputCalSource)
    ccall((:DAQmxAdjust1540Cal, nidaqmx), int32, (CalHandle, float64, float64, int32), calHandle, refVoltage, measOutput, inputCalSource)
end

function DAQmxConfigureTEDS(physicalChannel, filePath)
    ccall((:DAQmxConfigureTEDS, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), physicalChannel, filePath)
end

function DAQmxClearTEDS(physicalChannel)
    ccall((:DAQmxClearTEDS, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxWriteToTEDSFromArray(physicalChannel, bitStream, arraySize, basicTEDSOptions)
    ccall((:DAQmxWriteToTEDSFromArray, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt8}, uInt32, int32), physicalChannel, bitStream, arraySize, basicTEDSOptions)
end

function DAQmxWriteToTEDSFromFile(physicalChannel, filePath, basicTEDSOptions)
    ccall((:DAQmxWriteToTEDSFromFile, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, int32), physicalChannel, filePath, basicTEDSOptions)
end

function DAQmxWaitForNextSampleClock(taskHandle, timeout, isLate)
    ccall((:DAQmxWaitForNextSampleClock, nidaqmx), int32, (TaskHandle, float64, Ptr{bool32}), taskHandle, timeout, isLate)
end

function DAQmxResetRealTimeAttribute(taskHandle, attribute)
    ccall((:DAQmxResetRealTimeAttribute, nidaqmx), int32, (TaskHandle, int32), taskHandle, attribute)
end

function DAQmxIsReadOrWriteLate(errorCode)
    ccall((:DAQmxIsReadOrWriteLate, nidaqmx), bool32, (int32,), errorCode)
end

function DAQmxSaveTask(taskHandle, saveAs, author, options)
    ccall((:DAQmxSaveTask, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, saveAs, author, options)
end

function DAQmxSaveGlobalChan(taskHandle, channelName, saveAs, author, options)
    ccall((:DAQmxSaveGlobalChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channelName, saveAs, author, options)
end

function DAQmxSaveScale(scaleName, saveAs, author, options)
    ccall((:DAQmxSaveScale, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, Ptr{Cchar}, uInt32), scaleName, saveAs, author, options)
end

function DAQmxDeleteSavedTask(taskName)
    ccall((:DAQmxDeleteSavedTask, nidaqmx), int32, (Ptr{Cchar},), taskName)
end

function DAQmxDeleteSavedGlobalChan(channelName)
    ccall((:DAQmxDeleteSavedGlobalChan, nidaqmx), int32, (Ptr{Cchar},), channelName)
end

function DAQmxDeleteSavedScale(scaleName)
    ccall((:DAQmxDeleteSavedScale, nidaqmx), int32, (Ptr{Cchar},), scaleName)
end

function DAQmxSetAnalogPowerUpStatesWithOutputType(channelNames, stateArray, channelTypeArray, arraySize)
    ccall((:DAQmxSetAnalogPowerUpStatesWithOutputType, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, Ptr{int32}, uInt32), channelNames, stateArray, channelTypeArray, arraySize)
end

function DAQmxGetAnalogPowerUpStatesWithOutputType(channelNames, stateArray, channelTypeArray, arraySizePtr)
    ccall((:DAQmxGetAnalogPowerUpStatesWithOutputType, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, Ptr{int32}, Ptr{uInt32}), channelNames, stateArray, channelTypeArray, arraySizePtr)
end

function DAQmxSetDigitalLogicFamilyPowerUpState(deviceName, logicFamily)
    ccall((:DAQmxSetDigitalLogicFamilyPowerUpState, nidaqmx), int32, (Ptr{Cchar}, int32), deviceName, logicFamily)
end

function DAQmxGetDigitalLogicFamilyPowerUpState(deviceName, logicFamily)
    ccall((:DAQmxGetDigitalLogicFamilyPowerUpState, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), deviceName, logicFamily)
end

function DAQmxAddNetworkDevice(IPAddress, deviceName, attemptReservation, timeout, deviceNameOut, deviceNameOutBufferSize)
    ccall((:DAQmxAddNetworkDevice, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, bool32, float64, Ptr{Cchar}, uInt32), IPAddress, deviceName, attemptReservation, timeout, deviceNameOut, deviceNameOutBufferSize)
end

function DAQmxDeleteNetworkDevice(deviceName)
    ccall((:DAQmxDeleteNetworkDevice, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxReserveNetworkDevice(deviceName, overrideReservation)
    ccall((:DAQmxReserveNetworkDevice, nidaqmx), int32, (Ptr{Cchar}, bool32), deviceName, overrideReservation)
end

function DAQmxUnreserveNetworkDevice(deviceName)
    ccall((:DAQmxUnreserveNetworkDevice, nidaqmx), int32, (Ptr{Cchar},), deviceName)
end

function DAQmxAutoConfigureCDAQSyncConnections(chassisDevicesPorts, timeout)
    ccall((:DAQmxAutoConfigureCDAQSyncConnections, nidaqmx), int32, (Ptr{Cchar}, float64), chassisDevicesPorts, timeout)
end

function DAQmxGetAutoConfiguredCDAQSyncConnections(portList, portListSize)
    ccall((:DAQmxGetAutoConfiguredCDAQSyncConnections, nidaqmx), int32, (Ptr{Cchar}, uInt32), portList, portListSize)
end

function DAQmxAreConfiguredCDAQSyncPortsDisconnected(chassisDevicesPorts, timeout, disconnectedPortsExist)
    ccall((:DAQmxAreConfiguredCDAQSyncPortsDisconnected, nidaqmx), int32, (Ptr{Cchar}, float64, Ptr{bool32}), chassisDevicesPorts, timeout, disconnectedPortsExist)
end

function DAQmxGetDisconnectedCDAQSyncPorts(portList, portListSize)
    ccall((:DAQmxGetDisconnectedCDAQSyncPorts, nidaqmx), int32, (Ptr{Cchar}, uInt32), portList, portListSize)
end

function DAQmxAddCDAQSyncConnection(portList)
    ccall((:DAQmxAddCDAQSyncConnection, nidaqmx), int32, (Ptr{Cchar},), portList)
end

function DAQmxRemoveCDAQSyncConnection(portList)
    ccall((:DAQmxRemoveCDAQSyncConnection, nidaqmx), int32, (Ptr{Cchar},), portList)
end

function DAQmxGetErrorString(errorCode, errorString, bufferSize)
    ccall((:DAQmxGetErrorString, nidaqmx), int32, (int32, Ptr{Cchar}, uInt32), errorCode, errorString, bufferSize)
end

function DAQmxGetExtendedErrorInfo(errorString, bufferSize)
    ccall((:DAQmxGetExtendedErrorInfo, nidaqmx), int32, (Ptr{Cchar}, uInt32), errorString, bufferSize)
end

function DAQmxGetBufInputBufSize(taskHandle, data)
    ccall((:DAQmxGetBufInputBufSize, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetBufInputBufSize(taskHandle, data)
    ccall((:DAQmxSetBufInputBufSize, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetBufInputBufSize(taskHandle)
    ccall((:DAQmxResetBufInputBufSize, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetBufInputOnbrdBufSize(taskHandle, data)
    ccall((:DAQmxGetBufInputOnbrdBufSize, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetBufOutputBufSize(taskHandle, data)
    ccall((:DAQmxGetBufOutputBufSize, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetBufOutputBufSize(taskHandle, data)
    ccall((:DAQmxSetBufOutputBufSize, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetBufOutputBufSize(taskHandle)
    ccall((:DAQmxResetBufOutputBufSize, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetBufOutputOnbrdBufSize(taskHandle, data)
    ccall((:DAQmxGetBufOutputOnbrdBufSize, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetBufOutputOnbrdBufSize(taskHandle, data)
    ccall((:DAQmxSetBufOutputOnbrdBufSize, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetBufOutputOnbrdBufSize(taskHandle)
    ccall((:DAQmxResetBufOutputOnbrdBufSize, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSelfCalSupported(deviceName, data)
    ccall((:DAQmxGetSelfCalSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), deviceName, data)
end

function DAQmxGetSelfCalLastTemp(deviceName, data)
    ccall((:DAQmxGetSelfCalLastTemp, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), deviceName, data)
end

function DAQmxGetExtCalRecommendedInterval(deviceName, data)
    ccall((:DAQmxGetExtCalRecommendedInterval, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetExtCalLastTemp(deviceName, data)
    ccall((:DAQmxGetExtCalLastTemp, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), deviceName, data)
end

function DAQmxGetCalUserDefinedInfo(deviceName, data, bufferSize)
    ccall((:DAQmxGetCalUserDefinedInfo, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), deviceName, data, bufferSize)
end

function DAQmxSetCalUserDefinedInfo(deviceName, data)
    ccall((:DAQmxSetCalUserDefinedInfo, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), deviceName, data)
end

function DAQmxGetCalUserDefinedInfoMaxSize(deviceName, data)
    ccall((:DAQmxGetCalUserDefinedInfoMaxSize, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetCalDevTemp(deviceName, data)
    ccall((:DAQmxGetCalDevTemp, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), deviceName, data)
end

function DAQmxGetCalAccConnectionCount(deviceName, data)
    ccall((:DAQmxGetCalAccConnectionCount, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxSetCalAccConnectionCount(deviceName, data)
    ccall((:DAQmxSetCalAccConnectionCount, nidaqmx), int32, (Ptr{Cchar}, uInt32), deviceName, data)
end

function DAQmxGetCalRecommendedAccConnectionCountLimit(deviceName, data)
    ccall((:DAQmxGetCalRecommendedAccConnectionCountLimit, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetAIMax(taskHandle, channel, data)
    ccall((:DAQmxGetAIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIMax(taskHandle, channel, data)
    ccall((:DAQmxSetAIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIMax(taskHandle, channel)
    ccall((:DAQmxResetAIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIMin(taskHandle, channel, data)
    ccall((:DAQmxGetAIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIMin(taskHandle, channel, data)
    ccall((:DAQmxSetAIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIMin(taskHandle, channel)
    ccall((:DAQmxResetAIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAICustomScaleName(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAICustomScaleName(taskHandle, channel, data)
    ccall((:DAQmxSetAICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAICustomScaleName(taskHandle, channel)
    ccall((:DAQmxResetAICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIMeasType(taskHandle, channel, data)
    ccall((:DAQmxGetAIMeasType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAIVoltageUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIVoltageUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIVoltageUnits(taskHandle, channel)
    ccall((:DAQmxResetAIVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVoltagedBRef(taskHandle, channel, data)
    ccall((:DAQmxGetAIVoltagedBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIVoltagedBRef(taskHandle, channel, data)
    ccall((:DAQmxSetAIVoltagedBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIVoltagedBRef(taskHandle, channel)
    ccall((:DAQmxResetAIVoltagedBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVoltageACRMSUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIVoltageACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIVoltageACRMSUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIVoltageACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIVoltageACRMSUnits(taskHandle, channel)
    ccall((:DAQmxResetAIVoltageACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAITempUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAITempUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAITempUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAITempUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAITempUnits(taskHandle, channel)
    ccall((:DAQmxResetAITempUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmcplType(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmcplType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIThrmcplType(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmcplType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIThrmcplType(taskHandle, channel)
    ccall((:DAQmxResetAIThrmcplType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmcplScaleType(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmcplScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIThrmcplScaleType(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmcplScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIThrmcplScaleType(taskHandle, channel)
    ccall((:DAQmxResetAIThrmcplScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmcplCJCSrc(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmcplCJCSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAIThrmcplCJCVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmcplCJCVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmcplCJCVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmcplCJCVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmcplCJCVal(taskHandle, channel)
    ccall((:DAQmxResetAIThrmcplCJCVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmcplCJCChan(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAIThrmcplCJCChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxGetAIRTDType(taskHandle, channel, data)
    ccall((:DAQmxGetAIRTDType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIRTDType(taskHandle, channel, data)
    ccall((:DAQmxSetAIRTDType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIRTDType(taskHandle, channel)
    ccall((:DAQmxResetAIRTDType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRTDR0(taskHandle, channel, data)
    ccall((:DAQmxGetAIRTDR0, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRTDR0(taskHandle, channel, data)
    ccall((:DAQmxSetAIRTDR0, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRTDR0(taskHandle, channel)
    ccall((:DAQmxResetAIRTDR0, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRTDA(taskHandle, channel, data)
    ccall((:DAQmxGetAIRTDA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRTDA(taskHandle, channel, data)
    ccall((:DAQmxSetAIRTDA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRTDA(taskHandle, channel)
    ccall((:DAQmxResetAIRTDA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRTDB(taskHandle, channel, data)
    ccall((:DAQmxGetAIRTDB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRTDB(taskHandle, channel, data)
    ccall((:DAQmxSetAIRTDB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRTDB(taskHandle, channel)
    ccall((:DAQmxResetAIRTDB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRTDC(taskHandle, channel, data)
    ccall((:DAQmxGetAIRTDC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRTDC(taskHandle, channel, data)
    ccall((:DAQmxSetAIRTDC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRTDC(taskHandle, channel)
    ccall((:DAQmxResetAIRTDC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmstrA(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmstrA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmstrA(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmstrA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmstrA(taskHandle, channel)
    ccall((:DAQmxResetAIThrmstrA, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmstrB(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmstrB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmstrB(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmstrB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmstrB(taskHandle, channel)
    ccall((:DAQmxResetAIThrmstrB, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmstrC(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmstrC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmstrC(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmstrC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmstrC(taskHandle, channel)
    ccall((:DAQmxResetAIThrmstrC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmstrR1(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmstrR1, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmstrR1(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmstrR1, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmstrR1(taskHandle, channel)
    ccall((:DAQmxResetAIThrmstrR1, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIForceReadFromChan(taskHandle, channel, data)
    ccall((:DAQmxGetAIForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIForceReadFromChan(taskHandle, channel, data)
    ccall((:DAQmxSetAIForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIForceReadFromChan(taskHandle, channel)
    ccall((:DAQmxResetAIForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAICurrentUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAICurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAICurrentUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAICurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAICurrentUnits(taskHandle, channel)
    ccall((:DAQmxResetAICurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAICurrentACRMSUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAICurrentACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAICurrentACRMSUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAICurrentACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAICurrentACRMSUnits(taskHandle, channel)
    ccall((:DAQmxResetAICurrentACRMSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIStrainUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIStrainUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIStrainUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIStrainUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIStrainUnits(taskHandle, channel)
    ccall((:DAQmxResetAIStrainUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIStrainGageForceReadFromChan(taskHandle, channel, data)
    ccall((:DAQmxGetAIStrainGageForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIStrainGageForceReadFromChan(taskHandle, channel, data)
    ccall((:DAQmxSetAIStrainGageForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIStrainGageForceReadFromChan(taskHandle, channel)
    ccall((:DAQmxResetAIStrainGageForceReadFromChan, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIStrainGageGageFactor(taskHandle, channel, data)
    ccall((:DAQmxGetAIStrainGageGageFactor, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIStrainGageGageFactor(taskHandle, channel, data)
    ccall((:DAQmxSetAIStrainGageGageFactor, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIStrainGageGageFactor(taskHandle, channel)
    ccall((:DAQmxResetAIStrainGageGageFactor, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIStrainGagePoissonRatio(taskHandle, channel, data)
    ccall((:DAQmxGetAIStrainGagePoissonRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIStrainGagePoissonRatio(taskHandle, channel, data)
    ccall((:DAQmxSetAIStrainGagePoissonRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIStrainGagePoissonRatio(taskHandle, channel)
    ccall((:DAQmxResetAIStrainGagePoissonRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIStrainGageCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAIStrainGageCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIStrainGageCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAIStrainGageCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIStrainGageCfg(taskHandle, channel)
    ccall((:DAQmxResetAIStrainGageCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRosetteStrainGageRosetteType(taskHandle, channel, data)
    ccall((:DAQmxGetAIRosetteStrainGageRosetteType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAIRosetteStrainGageOrientation(taskHandle, channel, data)
    ccall((:DAQmxGetAIRosetteStrainGageOrientation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRosetteStrainGageOrientation(taskHandle, channel, data)
    ccall((:DAQmxSetAIRosetteStrainGageOrientation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRosetteStrainGageOrientation(taskHandle, channel)
    ccall((:DAQmxResetAIRosetteStrainGageOrientation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRosetteStrainGageStrainChans(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAIRosetteStrainGageStrainChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxGetAIRosetteStrainGageRosetteMeasType(taskHandle, channel, data)
    ccall((:DAQmxGetAIRosetteStrainGageRosetteMeasType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIRosetteStrainGageRosetteMeasType(taskHandle, channel, data)
    ccall((:DAQmxSetAIRosetteStrainGageRosetteMeasType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIRosetteStrainGageRosetteMeasType(taskHandle, channel)
    ccall((:DAQmxResetAIRosetteStrainGageRosetteMeasType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIResistanceUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIResistanceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIResistanceUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIResistanceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIResistanceUnits(taskHandle, channel)
    ccall((:DAQmxResetAIResistanceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIFreqUnits(taskHandle, channel)
    ccall((:DAQmxResetAIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFreqThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetAIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIFreqThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetAIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIFreqThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetAIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFreqHyst(taskHandle, channel, data)
    ccall((:DAQmxGetAIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIFreqHyst(taskHandle, channel, data)
    ccall((:DAQmxSetAIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIFreqHyst(taskHandle, channel)
    ccall((:DAQmxResetAIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILVDTUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAILVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAILVDTUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAILVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAILVDTUnits(taskHandle, channel)
    ccall((:DAQmxResetAILVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILVDTSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAILVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAILVDTSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAILVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAILVDTSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAILVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILVDTSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAILVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAILVDTSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAILVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAILVDTSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAILVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRVDTUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIRVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIRVDTUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIRVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIRVDTUnits(taskHandle, channel)
    ccall((:DAQmxResetAIRVDTUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRVDTSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIRVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRVDTSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIRVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRVDTSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIRVDTSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRVDTSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIRVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIRVDTSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIRVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIRVDTSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIRVDTSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIEddyCurrentProxProbeUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIEddyCurrentProxProbeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIEddyCurrentProxProbeUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIEddyCurrentProxProbeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIEddyCurrentProxProbeUnits(taskHandle, channel)
    ccall((:DAQmxResetAIEddyCurrentProxProbeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIEddyCurrentProxProbeSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIEddyCurrentProxProbeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIEddyCurrentProxProbeSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIEddyCurrentProxProbeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIEddyCurrentProxProbeSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIEddyCurrentProxProbeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIEddyCurrentProxProbeSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIEddyCurrentProxProbeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIEddyCurrentProxProbeSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIEddyCurrentProxProbeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIEddyCurrentProxProbeSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIEddyCurrentProxProbeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISoundPressureMaxSoundPressureLvl(taskHandle, channel, data)
    ccall((:DAQmxGetAISoundPressureMaxSoundPressureLvl, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAISoundPressureMaxSoundPressureLvl(taskHandle, channel, data)
    ccall((:DAQmxSetAISoundPressureMaxSoundPressureLvl, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAISoundPressureMaxSoundPressureLvl(taskHandle, channel)
    ccall((:DAQmxResetAISoundPressureMaxSoundPressureLvl, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISoundPressureUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAISoundPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAISoundPressureUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAISoundPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAISoundPressureUnits(taskHandle, channel)
    ccall((:DAQmxResetAISoundPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISoundPressuredBRef(taskHandle, channel, data)
    ccall((:DAQmxGetAISoundPressuredBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAISoundPressuredBRef(taskHandle, channel, data)
    ccall((:DAQmxSetAISoundPressuredBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAISoundPressuredBRef(taskHandle, channel)
    ccall((:DAQmxResetAISoundPressuredBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIMicrophoneSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIMicrophoneSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIMicrophoneSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIMicrophoneSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIMicrophoneSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIMicrophoneSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccelUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccelUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIAccelUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccelUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIAccelUnits(taskHandle, channel)
    ccall((:DAQmxResetAIAccelUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAcceldBRef(taskHandle, channel, data)
    ccall((:DAQmxGetAIAcceldBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIAcceldBRef(taskHandle, channel, data)
    ccall((:DAQmxSetAIAcceldBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIAcceldBRef(taskHandle, channel)
    ccall((:DAQmxResetAIAcceldBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccel4WireDCVoltageSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccel4WireDCVoltageSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIAccel4WireDCVoltageSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccel4WireDCVoltageSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIAccel4WireDCVoltageSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIAccel4WireDCVoltageSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccel4WireDCVoltageSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccel4WireDCVoltageSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIAccel4WireDCVoltageSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccel4WireDCVoltageSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIAccel4WireDCVoltageSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIAccel4WireDCVoltageSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccelSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccelSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIAccelSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccelSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIAccelSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIAccelSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccelSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccelSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIAccelSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccelSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIAccelSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIAccelSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccelChargeSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccelChargeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIAccelChargeSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccelChargeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIAccelChargeSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIAccelChargeSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAccelChargeSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIAccelChargeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIAccelChargeSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIAccelChargeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIAccelChargeSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIAccelChargeSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVelocityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIVelocityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIVelocityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIVelocityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIVelocityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIVelocityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVelocityIEPESensordBRef(taskHandle, channel, data)
    ccall((:DAQmxGetAIVelocityIEPESensordBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIVelocityIEPESensordBRef(taskHandle, channel, data)
    ccall((:DAQmxSetAIVelocityIEPESensordBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIVelocityIEPESensordBRef(taskHandle, channel)
    ccall((:DAQmxResetAIVelocityIEPESensordBRef, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVelocityIEPESensorSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIVelocityIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIVelocityIEPESensorSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIVelocityIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIVelocityIEPESensorSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIVelocityIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIVelocityIEPESensorSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIVelocityIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIVelocityIEPESensorSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIVelocityIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIVelocityIEPESensorSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIVelocityIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIForceUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIForceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIForceUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIForceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIForceUnits(taskHandle, channel)
    ccall((:DAQmxResetAIForceUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIForceIEPESensorSensitivity(taskHandle, channel, data)
    ccall((:DAQmxGetAIForceIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIForceIEPESensorSensitivity(taskHandle, channel, data)
    ccall((:DAQmxSetAIForceIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIForceIEPESensorSensitivity(taskHandle, channel)
    ccall((:DAQmxResetAIForceIEPESensorSensitivity, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIForceIEPESensorSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIForceIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIForceIEPESensorSensitivityUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIForceIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIForceIEPESensorSensitivityUnits(taskHandle, channel)
    ccall((:DAQmxResetAIForceIEPESensorSensitivityUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIPressureUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIPressureUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIPressureUnits(taskHandle, channel)
    ccall((:DAQmxResetAIPressureUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAITorqueUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAITorqueUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAITorqueUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAITorqueUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAITorqueUnits(taskHandle, channel)
    ccall((:DAQmxResetAITorqueUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeUnits(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeElectricalUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeElectricalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeElectricalUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeElectricalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeElectricalUnits(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeElectricalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgePhysicalUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgePhysicalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgePhysicalUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgePhysicalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgePhysicalUnits(taskHandle, channel)
    ccall((:DAQmxResetAIBridgePhysicalUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeScaleType(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeScaleType(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeScaleType(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTwoPointLinFirstElectricalVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeTwoPointLinFirstElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeTwoPointLinFirstElectricalVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeTwoPointLinFirstElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeTwoPointLinFirstElectricalVal(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTwoPointLinFirstElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTwoPointLinFirstPhysicalVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeTwoPointLinFirstPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeTwoPointLinFirstPhysicalVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeTwoPointLinFirstPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeTwoPointLinFirstPhysicalVal(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTwoPointLinFirstPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTwoPointLinSecondElectricalVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeTwoPointLinSecondElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeTwoPointLinSecondElectricalVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeTwoPointLinSecondElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeTwoPointLinSecondElectricalVal(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTwoPointLinSecondElectricalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTwoPointLinSecondPhysicalVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeTwoPointLinSecondPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeTwoPointLinSecondPhysicalVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeTwoPointLinSecondPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeTwoPointLinSecondPhysicalVal(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTwoPointLinSecondPhysicalVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTableElectricalVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIBridgeTableElectricalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIBridgeTableElectricalVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIBridgeTableElectricalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIBridgeTableElectricalVals(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTableElectricalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeTablePhysicalVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIBridgeTablePhysicalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIBridgeTablePhysicalVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIBridgeTablePhysicalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIBridgeTablePhysicalVals(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeTablePhysicalVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgePolyForwardCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIBridgePolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIBridgePolyForwardCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIBridgePolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIBridgePolyForwardCoeff(taskHandle, channel)
    ccall((:DAQmxResetAIBridgePolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgePolyReverseCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIBridgePolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIBridgePolyReverseCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIBridgePolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIBridgePolyReverseCoeff(taskHandle, channel)
    ccall((:DAQmxResetAIBridgePolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChargeUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIChargeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIChargeUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIChargeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIChargeUnits(taskHandle, channel)
    ccall((:DAQmxResetAIChargeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIIsTEDS(taskHandle, channel, data)
    ccall((:DAQmxGetAIIsTEDS, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetAITEDSUnits(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAITEDSUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxGetAICoupling(taskHandle, channel, data)
    ccall((:DAQmxGetAICoupling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAICoupling(taskHandle, channel, data)
    ccall((:DAQmxSetAICoupling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAICoupling(taskHandle, channel)
    ccall((:DAQmxResetAICoupling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIImpedance(taskHandle, channel, data)
    ccall((:DAQmxGetAIImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIImpedance(taskHandle, channel, data)
    ccall((:DAQmxSetAIImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIImpedance(taskHandle, channel)
    ccall((:DAQmxResetAIImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAITermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAITermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAITermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAITermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAITermCfg(taskHandle, channel)
    ccall((:DAQmxResetAITermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIInputSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAIInputSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAIInputSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAIInputSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAIInputSrc(taskHandle, channel)
    ccall((:DAQmxResetAIInputSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIResistanceCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAIResistanceCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIResistanceCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAIResistanceCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIResistanceCfg(taskHandle, channel)
    ccall((:DAQmxResetAIResistanceCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILeadWireResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAILeadWireResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAILeadWireResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAILeadWireResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAILeadWireResistance(taskHandle, channel)
    ccall((:DAQmxResetAILeadWireResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeCfg(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeNomResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeNomResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeNomResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeNomResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeNomResistance(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeNomResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeInitialVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeInitialVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeInitialVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeInitialVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeInitialVoltage(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeInitialVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeInitialRatio(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeInitialRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeInitialRatio(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeInitialRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeInitialRatio(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeInitialRatio, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalEnable(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalSelect(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalSelect, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalSelect(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalSelect, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalSelect(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalSelect, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalShuntCalASrc(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalShuntCalASrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalShuntCalASrc(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalShuntCalASrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalShuntCalASrc(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalShuntCalASrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalGainAdjust(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalGainAdjust, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalGainAdjust(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalGainAdjust, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalGainAdjust(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalGainAdjust, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalShuntCalAResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalShuntCalAResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalShuntCalAResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalShuntCalAResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalShuntCalAResistance(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalShuntCalAResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalShuntCalAActualResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalShuntCalAActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalShuntCalAActualResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalShuntCalAActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalShuntCalAActualResistance(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalShuntCalAActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalShuntCalBResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalShuntCalBResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalShuntCalBResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalShuntCalBResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalShuntCalBResistance(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalShuntCalBResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeShuntCalShuntCalBActualResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeShuntCalShuntCalBActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeShuntCalShuntCalBActualResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeShuntCalShuntCalBActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIBridgeShuntCalShuntCalBActualResistance(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeShuntCalShuntCalBActualResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeBalanceCoarsePot(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeBalanceCoarsePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeBalanceCoarsePot(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeBalanceCoarsePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeBalanceCoarsePot(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeBalanceCoarsePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIBridgeBalanceFinePot(taskHandle, channel, data)
    ccall((:DAQmxGetAIBridgeBalanceFinePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIBridgeBalanceFinePot(taskHandle, channel, data)
    ccall((:DAQmxSetAIBridgeBalanceFinePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIBridgeBalanceFinePot(taskHandle, channel)
    ccall((:DAQmxResetAIBridgeBalanceFinePot, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAICurrentShuntLoc(taskHandle, channel, data)
    ccall((:DAQmxGetAICurrentShuntLoc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAICurrentShuntLoc(taskHandle, channel, data)
    ccall((:DAQmxSetAICurrentShuntLoc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAICurrentShuntLoc(taskHandle, channel)
    ccall((:DAQmxResetAICurrentShuntLoc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAICurrentShuntResistance(taskHandle, channel, data)
    ccall((:DAQmxGetAICurrentShuntResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAICurrentShuntResistance(taskHandle, channel, data)
    ccall((:DAQmxSetAICurrentShuntResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAICurrentShuntResistance(taskHandle, channel)
    ccall((:DAQmxResetAICurrentShuntResistance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitSense(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitSense(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIExcitSense(taskHandle, channel)
    ccall((:DAQmxResetAIExcitSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitSrc(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIExcitSrc(taskHandle, channel)
    ccall((:DAQmxResetAIExcitSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIExcitVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIExcitVal(taskHandle, channel)
    ccall((:DAQmxResetAIExcitVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitUseForScaling(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitUseForScaling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitUseForScaling(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitUseForScaling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIExcitUseForScaling(taskHandle, channel)
    ccall((:DAQmxResetAIExcitUseForScaling, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitUseMultiplexed(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitUseMultiplexed, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitUseMultiplexed(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitUseMultiplexed, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIExcitUseMultiplexed(taskHandle, channel)
    ccall((:DAQmxResetAIExcitUseMultiplexed, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitActualVal(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitActualVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIExcitActualVal(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitActualVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIExcitActualVal(taskHandle, channel)
    ccall((:DAQmxResetAIExcitActualVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitDCorAC(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitDCorAC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitDCorAC(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitDCorAC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIExcitDCorAC(taskHandle, channel)
    ccall((:DAQmxResetAIExcitDCorAC, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitVoltageOrCurrent(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitVoltageOrCurrent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitVoltageOrCurrent(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitVoltageOrCurrent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIExcitVoltageOrCurrent(taskHandle, channel)
    ccall((:DAQmxResetAIExcitVoltageOrCurrent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIExcitIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetAIExcitIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIExcitIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetAIExcitIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIExcitIdleOutputBehavior(taskHandle, channel)
    ccall((:DAQmxResetAIExcitIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIACExcitFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIACExcitFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIACExcitFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIACExcitFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIACExcitFreq(taskHandle, channel)
    ccall((:DAQmxResetAIACExcitFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIACExcitSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIACExcitSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIACExcitSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIACExcitSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIACExcitSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetAIACExcitSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIACExcitWireMode(taskHandle, channel, data)
    ccall((:DAQmxGetAIACExcitWireMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIACExcitWireMode(taskHandle, channel, data)
    ccall((:DAQmxSetAIACExcitWireMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIACExcitWireMode(taskHandle, channel)
    ccall((:DAQmxResetAIACExcitWireMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISensorPowerVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetAISensorPowerVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAISensorPowerVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetAISensorPowerVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAISensorPowerVoltage(taskHandle, channel)
    ccall((:DAQmxResetAISensorPowerVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISensorPowerCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAISensorPowerCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAISensorPowerCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAISensorPowerCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAISensorPowerCfg(taskHandle, channel)
    ccall((:DAQmxResetAISensorPowerCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISensorPowerType(taskHandle, channel, data)
    ccall((:DAQmxGetAISensorPowerType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAISensorPowerType(taskHandle, channel, data)
    ccall((:DAQmxSetAISensorPowerType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAISensorPowerType(taskHandle, channel)
    ccall((:DAQmxResetAISensorPowerType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIOpenThrmcplDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIOpenThrmcplDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIOpenThrmcplDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIOpenThrmcplDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIOpenThrmcplDetectEnable(taskHandle, channel)
    ccall((:DAQmxResetAIOpenThrmcplDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIThrmcplLeadOffsetVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetAIThrmcplLeadOffsetVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIThrmcplLeadOffsetVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetAIThrmcplLeadOffsetVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIThrmcplLeadOffsetVoltage(taskHandle, channel)
    ccall((:DAQmxResetAIThrmcplLeadOffsetVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAtten(taskHandle, channel, data)
    ccall((:DAQmxGetAIAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIAtten(taskHandle, channel, data)
    ccall((:DAQmxSetAIAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIAtten(taskHandle, channel)
    ccall((:DAQmxResetAIAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIProbeAtten(taskHandle, channel, data)
    ccall((:DAQmxGetAIProbeAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIProbeAtten(taskHandle, channel, data)
    ccall((:DAQmxSetAIProbeAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIProbeAtten(taskHandle, channel)
    ccall((:DAQmxResetAIProbeAtten, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAILowpassEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAILowpassEnable(taskHandle, channel)
    ccall((:DAQmxResetAILowpassEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAILowpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAILowpassCutoffFreq(taskHandle, channel)
    ccall((:DAQmxResetAILowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassSwitchCapClkSrc(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassSwitchCapClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAILowpassSwitchCapClkSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassSwitchCapClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAILowpassSwitchCapClkSrc(taskHandle, channel)
    ccall((:DAQmxResetAILowpassSwitchCapClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassSwitchCapExtClkFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassSwitchCapExtClkFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAILowpassSwitchCapExtClkFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassSwitchCapExtClkFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAILowpassSwitchCapExtClkFreq(taskHandle, channel)
    ccall((:DAQmxResetAILowpassSwitchCapExtClkFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassSwitchCapExtClkDiv(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassSwitchCapExtClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAILowpassSwitchCapExtClkDiv(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassSwitchCapExtClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAILowpassSwitchCapExtClkDiv(taskHandle, channel)
    ccall((:DAQmxResetAILowpassSwitchCapExtClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILowpassSwitchCapOutClkDiv(taskHandle, channel, data)
    ccall((:DAQmxGetAILowpassSwitchCapOutClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAILowpassSwitchCapOutClkDiv(taskHandle, channel, data)
    ccall((:DAQmxSetAILowpassSwitchCapOutClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAILowpassSwitchCapOutClkDiv(taskHandle, channel)
    ccall((:DAQmxResetAILowpassSwitchCapOutClkDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrType(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrType(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrType(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrResponse(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrResponse(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrResponse(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrOrder(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrOrder(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrOrder(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrLowpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrLowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrLowpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrLowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrLowpassCutoffFreq(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrLowpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrHighpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrHighpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrHighpassCutoffFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrHighpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrHighpassCutoffFreq(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrHighpassCutoffFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrBandpassCenterFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrBandpassCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrBandpassCenterFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrBandpassCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrBandpassCenterFreq(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrBandpassCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrBandpassWidth(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrBandpassWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrBandpassWidth(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrBandpassWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrBandpassWidth(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrBandpassWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrNotchCenterFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrNotchCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrNotchCenterFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrNotchCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrNotchCenterFreq(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrNotchCenterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrNotchWidth(taskHandle, channel, data)
    ccall((:DAQmxGetAIDigFltrNotchWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDigFltrNotchWidth(taskHandle, channel, data)
    ccall((:DAQmxSetAIDigFltrNotchWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDigFltrNotchWidth(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrNotchWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDigFltrCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIDigFltrCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIDigFltrCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIDigFltrCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIDigFltrCoeff(taskHandle, channel)
    ccall((:DAQmxResetAIDigFltrCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIFilterEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIFilterEnable(taskHandle, channel)
    ccall((:DAQmxResetAIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIFilterFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIFilterFreq(taskHandle, channel)
    ccall((:DAQmxResetAIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterResponse(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIFilterResponse(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIFilterResponse(taskHandle, channel)
    ccall((:DAQmxResetAIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterOrder(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIFilterOrder(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIFilterOrder(taskHandle, channel)
    ccall((:DAQmxResetAIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxGetAIFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIFilterDelayUnits(taskHandle, channel)
    ccall((:DAQmxResetAIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRemoveFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxGetAIRemoveFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIRemoveFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxSetAIRemoveFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIRemoveFilterDelay(taskHandle, channel)
    ccall((:DAQmxResetAIRemoveFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIFilterDelayAdjustment(taskHandle, channel, data)
    ccall((:DAQmxGetAIFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIFilterDelayAdjustment(taskHandle, channel, data)
    ccall((:DAQmxSetAIFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIFilterDelayAdjustment(taskHandle, channel)
    ccall((:DAQmxResetAIFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAveragingWinSize(taskHandle, channel, data)
    ccall((:DAQmxGetAIAveragingWinSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIAveragingWinSize(taskHandle, channel, data)
    ccall((:DAQmxSetAIAveragingWinSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIAveragingWinSize(taskHandle, channel)
    ccall((:DAQmxResetAIAveragingWinSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIResolutionUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAIResolutionUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAIResolution(taskHandle, channel, data)
    ccall((:DAQmxGetAIResolution, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxGetAIRawSampSize(taskHandle, channel, data)
    ccall((:DAQmxGetAIRawSampSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetAIRawSampJustification(taskHandle, channel, data)
    ccall((:DAQmxGetAIRawSampJustification, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAIADCTimingMode(taskHandle, channel, data)
    ccall((:DAQmxGetAIADCTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIADCTimingMode(taskHandle, channel, data)
    ccall((:DAQmxSetAIADCTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIADCTimingMode(taskHandle, channel)
    ccall((:DAQmxResetAIADCTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIADCCustomTimingMode(taskHandle, channel, data)
    ccall((:DAQmxGetAIADCCustomTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIADCCustomTimingMode(taskHandle, channel, data)
    ccall((:DAQmxSetAIADCCustomTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIADCCustomTimingMode(taskHandle, channel)
    ccall((:DAQmxResetAIADCCustomTimingMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDitherEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIDitherEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIDitherEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIDitherEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIDitherEnable(taskHandle, channel)
    ccall((:DAQmxResetAIDitherEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalHasValidCalInfo(taskHandle, channel, data)
    ccall((:DAQmxGetAIChanCalHasValidCalInfo, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetAIChanCalEnableCal(taskHandle, channel, data)
    ccall((:DAQmxGetAIChanCalEnableCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIChanCalEnableCal(taskHandle, channel, data)
    ccall((:DAQmxSetAIChanCalEnableCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIChanCalEnableCal(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalEnableCal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalApplyCalIfExp(taskHandle, channel, data)
    ccall((:DAQmxGetAIChanCalApplyCalIfExp, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIChanCalApplyCalIfExp(taskHandle, channel, data)
    ccall((:DAQmxSetAIChanCalApplyCalIfExp, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIChanCalApplyCalIfExp(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalApplyCalIfExp, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalScaleType(taskHandle, channel, data)
    ccall((:DAQmxGetAIChanCalScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIChanCalScaleType(taskHandle, channel, data)
    ccall((:DAQmxSetAIChanCalScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIChanCalScaleType(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalScaleType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalTablePreScaledVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalTablePreScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalTablePreScaledVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalTablePreScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalTablePreScaledVals(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalTablePreScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalTableScaledVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalTableScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalTableScaledVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalTableScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalTableScaledVals(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalTableScaledVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalPolyForwardCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalPolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalPolyForwardCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalPolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalPolyForwardCoeff(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalPolyForwardCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalPolyReverseCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalPolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalPolyReverseCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalPolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalPolyReverseCoeff(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalPolyReverseCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalOperatorName(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAIChanCalOperatorName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAIChanCalOperatorName(taskHandle, channel, data)
    ccall((:DAQmxSetAIChanCalOperatorName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAIChanCalOperatorName(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalOperatorName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalDesc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAIChanCalDesc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAIChanCalDesc(taskHandle, channel, data)
    ccall((:DAQmxSetAIChanCalDesc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAIChanCalDesc(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalDesc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalVerifRefVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalVerifRefVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalVerifRefVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalVerifRefVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalVerifRefVals(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalVerifRefVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChanCalVerifAcqVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIChanCalVerifAcqVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxSetAIChanCalVerifAcqVals(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxSetAIChanCalVerifAcqVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxResetAIChanCalVerifAcqVals(taskHandle, channel)
    ccall((:DAQmxResetAIChanCalVerifAcqVals, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRngHigh(taskHandle, channel, data)
    ccall((:DAQmxGetAIRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRngHigh(taskHandle, channel, data)
    ccall((:DAQmxSetAIRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRngHigh(taskHandle, channel)
    ccall((:DAQmxResetAIRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRngLow(taskHandle, channel, data)
    ccall((:DAQmxGetAIRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIRngLow(taskHandle, channel, data)
    ccall((:DAQmxSetAIRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIRngLow(taskHandle, channel)
    ccall((:DAQmxResetAIRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDCOffset(taskHandle, channel, data)
    ccall((:DAQmxGetAIDCOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDCOffset(taskHandle, channel, data)
    ccall((:DAQmxSetAIDCOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDCOffset(taskHandle, channel)
    ccall((:DAQmxResetAIDCOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIGain(taskHandle, channel, data)
    ccall((:DAQmxGetAIGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIGain(taskHandle, channel, data)
    ccall((:DAQmxSetAIGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIGain(taskHandle, channel)
    ccall((:DAQmxResetAIGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAISampAndHoldEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAISampAndHoldEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAISampAndHoldEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAISampAndHoldEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAISampAndHoldEnable(taskHandle, channel)
    ccall((:DAQmxResetAISampAndHoldEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIAutoZeroMode(taskHandle, channel, data)
    ccall((:DAQmxGetAIAutoZeroMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIAutoZeroMode(taskHandle, channel, data)
    ccall((:DAQmxSetAIAutoZeroMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIAutoZeroMode(taskHandle, channel)
    ccall((:DAQmxResetAIAutoZeroMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIChopEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIChopEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIChopEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIChopEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIChopEnable(taskHandle, channel)
    ccall((:DAQmxResetAIChopEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDataXferMaxRate(taskHandle, channel, data)
    ccall((:DAQmxGetAIDataXferMaxRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIDataXferMaxRate(taskHandle, channel, data)
    ccall((:DAQmxSetAIDataXferMaxRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIDataXferMaxRate(taskHandle, channel)
    ccall((:DAQmxResetAIDataXferMaxRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetAIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetAIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIDataXferMech(taskHandle, channel)
    ccall((:DAQmxResetAIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetAIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetAIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIDataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetAIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDataXferCustomThreshold(taskHandle, channel, data)
    ccall((:DAQmxGetAIDataXferCustomThreshold, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIDataXferCustomThreshold(taskHandle, channel, data)
    ccall((:DAQmxSetAIDataXferCustomThreshold, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIDataXferCustomThreshold(taskHandle, channel)
    ccall((:DAQmxResetAIDataXferCustomThreshold, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetAIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetAIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetAIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetAIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetAIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAIUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetAIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetAIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIRawDataCompressionType(taskHandle, channel, data)
    ccall((:DAQmxGetAIRawDataCompressionType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAIRawDataCompressionType(taskHandle, channel, data)
    ccall((:DAQmxSetAIRawDataCompressionType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAIRawDataCompressionType(taskHandle, channel)
    ccall((:DAQmxResetAIRawDataCompressionType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAILossyLSBRemovalCompressedSampSize(taskHandle, channel, data)
    ccall((:DAQmxGetAILossyLSBRemovalCompressedSampSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAILossyLSBRemovalCompressedSampSize(taskHandle, channel, data)
    ccall((:DAQmxSetAILossyLSBRemovalCompressedSampSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAILossyLSBRemovalCompressedSampSize(taskHandle, channel)
    ccall((:DAQmxResetAILossyLSBRemovalCompressedSampSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIDevScalingCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAIDevScalingCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxGetAIEnhancedAliasRejectionEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIEnhancedAliasRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIEnhancedAliasRejectionEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIEnhancedAliasRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIEnhancedAliasRejectionEnable(taskHandle, channel)
    ccall((:DAQmxResetAIEnhancedAliasRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIOpenChanDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIOpenChanDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIOpenChanDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIOpenChanDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIOpenChanDetectEnable(taskHandle, channel)
    ccall((:DAQmxResetAIOpenChanDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIInputLimitsFaultDetectUpperLimit(taskHandle, channel, data)
    ccall((:DAQmxGetAIInputLimitsFaultDetectUpperLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIInputLimitsFaultDetectUpperLimit(taskHandle, channel, data)
    ccall((:DAQmxSetAIInputLimitsFaultDetectUpperLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIInputLimitsFaultDetectUpperLimit(taskHandle, channel)
    ccall((:DAQmxResetAIInputLimitsFaultDetectUpperLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIInputLimitsFaultDetectLowerLimit(taskHandle, channel, data)
    ccall((:DAQmxGetAIInputLimitsFaultDetectLowerLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAIInputLimitsFaultDetectLowerLimit(taskHandle, channel, data)
    ccall((:DAQmxSetAIInputLimitsFaultDetectLowerLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAIInputLimitsFaultDetectLowerLimit(taskHandle, channel)
    ccall((:DAQmxResetAIInputLimitsFaultDetectLowerLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIInputLimitsFaultDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIInputLimitsFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIInputLimitsFaultDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIInputLimitsFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIInputLimitsFaultDetectEnable(taskHandle, channel)
    ccall((:DAQmxResetAIInputLimitsFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIPowerSupplyFaultDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIPowerSupplyFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIPowerSupplyFaultDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIPowerSupplyFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIPowerSupplyFaultDetectEnable(taskHandle, channel)
    ccall((:DAQmxResetAIPowerSupplyFaultDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAIOvercurrentDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAIOvercurrentDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAIOvercurrentDetectEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAIOvercurrentDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAIOvercurrentDetectEnable(taskHandle, channel)
    ccall((:DAQmxResetAIOvercurrentDetectEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOMax(taskHandle, channel, data)
    ccall((:DAQmxGetAOMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOMax(taskHandle, channel, data)
    ccall((:DAQmxSetAOMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOMax(taskHandle, channel)
    ccall((:DAQmxResetAOMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOMin(taskHandle, channel, data)
    ccall((:DAQmxGetAOMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOMin(taskHandle, channel, data)
    ccall((:DAQmxSetAOMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOMin(taskHandle, channel)
    ccall((:DAQmxResetAOMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOCustomScaleName(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAOCustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAOCustomScaleName(taskHandle, channel, data)
    ccall((:DAQmxSetAOCustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAOCustomScaleName(taskHandle, channel)
    ccall((:DAQmxResetAOCustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOOutputType(taskHandle, channel, data)
    ccall((:DAQmxGetAOOutputType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetAOVoltageUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAOVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOVoltageUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAOVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOVoltageUnits(taskHandle, channel)
    ccall((:DAQmxResetAOVoltageUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOVoltageCurrentLimit(taskHandle, channel, data)
    ccall((:DAQmxGetAOVoltageCurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOVoltageCurrentLimit(taskHandle, channel, data)
    ccall((:DAQmxSetAOVoltageCurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOVoltageCurrentLimit(taskHandle, channel)
    ccall((:DAQmxResetAOVoltageCurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOCurrentUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAOCurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOCurrentUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAOCurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOCurrentUnits(taskHandle, channel)
    ccall((:DAQmxResetAOCurrentUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenType(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenType(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenType(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenFreq(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenFreq(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenFreq(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenAmplitude(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenAmplitude, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenAmplitude(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenAmplitude, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenAmplitude(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenAmplitude, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenOffset(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenOffset(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenOffset(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenStartPhase(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenStartPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenStartPhase(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenStartPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenStartPhase(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenStartPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenSquareDutyCycle(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenSquareDutyCycle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenSquareDutyCycle(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenSquareDutyCycle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenSquareDutyCycle(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenSquareDutyCycle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenModulationType(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenModulationType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenModulationType(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenModulationType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenModulationType(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenModulationType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFuncGenFMDeviation(taskHandle, channel, data)
    ccall((:DAQmxGetAOFuncGenFMDeviation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFuncGenFMDeviation(taskHandle, channel, data)
    ccall((:DAQmxSetAOFuncGenFMDeviation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFuncGenFMDeviation(taskHandle, channel)
    ccall((:DAQmxResetAOFuncGenFMDeviation, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOOutputImpedance(taskHandle, channel, data)
    ccall((:DAQmxGetAOOutputImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOOutputImpedance(taskHandle, channel, data)
    ccall((:DAQmxSetAOOutputImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOOutputImpedance(taskHandle, channel)
    ccall((:DAQmxResetAOOutputImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOLoadImpedance(taskHandle, channel, data)
    ccall((:DAQmxGetAOLoadImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOLoadImpedance(taskHandle, channel, data)
    ccall((:DAQmxSetAOLoadImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOLoadImpedance(taskHandle, channel)
    ccall((:DAQmxResetAOLoadImpedance, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetAOIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetAOIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOIdleOutputBehavior(taskHandle, channel)
    ccall((:DAQmxResetAOIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetAOTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetAOTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOTermCfg(taskHandle, channel)
    ccall((:DAQmxResetAOTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOCommonModeOffset(taskHandle, channel, data)
    ccall((:DAQmxGetAOCommonModeOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOCommonModeOffset(taskHandle, channel, data)
    ccall((:DAQmxSetAOCommonModeOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOCommonModeOffset(taskHandle, channel)
    ccall((:DAQmxResetAOCommonModeOffset, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOResolutionUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAOResolutionUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOResolutionUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAOResolutionUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOResolutionUnits(taskHandle, channel)
    ccall((:DAQmxResetAOResolutionUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOResolution(taskHandle, channel, data)
    ccall((:DAQmxGetAOResolution, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxGetAODACRngHigh(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAODACRngHigh(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAODACRngHigh(taskHandle, channel)
    ccall((:DAQmxResetAODACRngHigh, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRngLow(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAODACRngLow(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAODACRngLow(taskHandle, channel)
    ccall((:DAQmxResetAODACRngLow, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRefConnToGnd(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRefConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAODACRefConnToGnd(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRefConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAODACRefConnToGnd(taskHandle, channel)
    ccall((:DAQmxResetAODACRefConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRefAllowConnToGnd(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRefAllowConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAODACRefAllowConnToGnd(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRefAllowConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAODACRefAllowConnToGnd(taskHandle, channel)
    ccall((:DAQmxResetAODACRefAllowConnToGnd, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRefSrc(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRefSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAODACRefSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRefSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAODACRefSrc(taskHandle, channel)
    ccall((:DAQmxResetAODACRefSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRefExtSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAODACRefExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAODACRefExtSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRefExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAODACRefExtSrc(taskHandle, channel)
    ccall((:DAQmxResetAODACRefExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACRefVal(taskHandle, channel, data)
    ccall((:DAQmxGetAODACRefVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAODACRefVal(taskHandle, channel, data)
    ccall((:DAQmxSetAODACRefVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAODACRefVal(taskHandle, channel)
    ccall((:DAQmxResetAODACRefVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACOffsetSrc(taskHandle, channel, data)
    ccall((:DAQmxGetAODACOffsetSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAODACOffsetSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAODACOffsetSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAODACOffsetSrc(taskHandle, channel)
    ccall((:DAQmxResetAODACOffsetSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACOffsetExtSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetAODACOffsetExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetAODACOffsetExtSrc(taskHandle, channel, data)
    ccall((:DAQmxSetAODACOffsetExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetAODACOffsetExtSrc(taskHandle, channel)
    ccall((:DAQmxResetAODACOffsetExtSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODACOffsetVal(taskHandle, channel, data)
    ccall((:DAQmxGetAODACOffsetVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAODACOffsetVal(taskHandle, channel, data)
    ccall((:DAQmxSetAODACOffsetVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAODACOffsetVal(taskHandle, channel)
    ccall((:DAQmxResetAODACOffsetVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOReglitchEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAOReglitchEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAOReglitchEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAOReglitchEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAOReglitchEnable(taskHandle, channel)
    ccall((:DAQmxResetAOReglitchEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxGetAOFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxSetAOFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFilterDelay(taskHandle, channel)
    ccall((:DAQmxResetAOFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxGetAOFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAOFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxSetAOFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAOFilterDelayUnits(taskHandle, channel)
    ccall((:DAQmxResetAOFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOFilterDelayAdjustment(taskHandle, channel, data)
    ccall((:DAQmxGetAOFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOFilterDelayAdjustment(taskHandle, channel, data)
    ccall((:DAQmxSetAOFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOFilterDelayAdjustment(taskHandle, channel)
    ccall((:DAQmxResetAOFilterDelayAdjustment, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOGain(taskHandle, channel, data)
    ccall((:DAQmxGetAOGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetAOGain(taskHandle, channel, data)
    ccall((:DAQmxSetAOGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetAOGain(taskHandle, channel)
    ccall((:DAQmxResetAOGain, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxGetAOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxSetAOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAOUseOnlyOnBrdMem(taskHandle, channel)
    ccall((:DAQmxResetAOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetAODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetAODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAODataXferMech(taskHandle, channel)
    ccall((:DAQmxResetAODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetAODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetAODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetAODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetAODataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetAODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetAOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetAOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAOUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetAOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetAOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetAOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetAOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetAOUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetAOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAOMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetAOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetAODevScalingCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetAODevScalingCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxGetAOEnhancedImageRejectionEnable(taskHandle, channel, data)
    ccall((:DAQmxGetAOEnhancedImageRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetAOEnhancedImageRejectionEnable(taskHandle, channel, data)
    ccall((:DAQmxSetAOEnhancedImageRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetAOEnhancedImageRejectionEnable(taskHandle, channel)
    ccall((:DAQmxResetAOEnhancedImageRejectionEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIInvertLines(taskHandle, channel, data)
    ccall((:DAQmxGetDIInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDIInvertLines(taskHandle, channel, data)
    ccall((:DAQmxSetDIInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDIInvertLines(taskHandle, channel)
    ccall((:DAQmxResetDIInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDINumLines(taskHandle, channel, data)
    ccall((:DAQmxGetDINumLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetDIDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetDIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDIDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDIDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetDIDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetDIDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetDIDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetDIDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetDIDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDigFltrEnableBusMode(taskHandle, channel, data)
    ccall((:DAQmxGetDIDigFltrEnableBusMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDIDigFltrEnableBusMode(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigFltrEnableBusMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDIDigFltrEnableBusMode(taskHandle, channel)
    ccall((:DAQmxResetDIDigFltrEnableBusMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetDIDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetDIDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetDIDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetDIDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetDIDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetDIDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetDIDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetDIDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetDIDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDIDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetDIDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDIDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetDIDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDITristate(taskHandle, channel, data)
    ccall((:DAQmxGetDITristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDITristate(taskHandle, channel, data)
    ccall((:DAQmxSetDITristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDITristate(taskHandle, channel)
    ccall((:DAQmxResetDITristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDILogicFamily(taskHandle, channel, data)
    ccall((:DAQmxGetDILogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDILogicFamily(taskHandle, channel, data)
    ccall((:DAQmxSetDILogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDILogicFamily(taskHandle, channel)
    ccall((:DAQmxResetDILogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetDIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetDIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDIDataXferMech(taskHandle, channel)
    ccall((:DAQmxResetDIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetDIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetDIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDIDataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetDIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetDIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetDIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetDIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetDIUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetDIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetDIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetDIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetDIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetDIUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetDIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetDIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetDIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDIMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetDIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDIAcquireOn(taskHandle, channel, data)
    ccall((:DAQmxGetDIAcquireOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDIAcquireOn(taskHandle, channel, data)
    ccall((:DAQmxSetDIAcquireOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDIAcquireOn(taskHandle, channel)
    ccall((:DAQmxResetDIAcquireOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOOutputDriveType(taskHandle, channel, data)
    ccall((:DAQmxGetDOOutputDriveType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOOutputDriveType(taskHandle, channel, data)
    ccall((:DAQmxSetDOOutputDriveType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOOutputDriveType(taskHandle, channel)
    ccall((:DAQmxResetDOOutputDriveType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOInvertLines(taskHandle, channel, data)
    ccall((:DAQmxGetDOInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDOInvertLines(taskHandle, channel, data)
    ccall((:DAQmxSetDOInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDOInvertLines(taskHandle, channel)
    ccall((:DAQmxResetDOInvertLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDONumLines(taskHandle, channel, data)
    ccall((:DAQmxGetDONumLines, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetDOTristate(taskHandle, channel, data)
    ccall((:DAQmxGetDOTristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDOTristate(taskHandle, channel, data)
    ccall((:DAQmxSetDOTristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDOTristate(taskHandle, channel)
    ccall((:DAQmxResetDOTristate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOLineStatesStartState(taskHandle, channel, data)
    ccall((:DAQmxGetDOLineStatesStartState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOLineStatesStartState(taskHandle, channel, data)
    ccall((:DAQmxSetDOLineStatesStartState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOLineStatesStartState(taskHandle, channel)
    ccall((:DAQmxResetDOLineStatesStartState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOLineStatesPausedState(taskHandle, channel, data)
    ccall((:DAQmxGetDOLineStatesPausedState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOLineStatesPausedState(taskHandle, channel, data)
    ccall((:DAQmxSetDOLineStatesPausedState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOLineStatesPausedState(taskHandle, channel)
    ccall((:DAQmxResetDOLineStatesPausedState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOLineStatesDoneState(taskHandle, channel, data)
    ccall((:DAQmxGetDOLineStatesDoneState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOLineStatesDoneState(taskHandle, channel, data)
    ccall((:DAQmxSetDOLineStatesDoneState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOLineStatesDoneState(taskHandle, channel)
    ccall((:DAQmxResetDOLineStatesDoneState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOLogicFamily(taskHandle, channel, data)
    ccall((:DAQmxGetDOLogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOLogicFamily(taskHandle, channel, data)
    ccall((:DAQmxSetDOLogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOLogicFamily(taskHandle, channel)
    ccall((:DAQmxResetDOLogicFamily, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOOvercurrentLimit(taskHandle, channel, data)
    ccall((:DAQmxGetDOOvercurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetDOOvercurrentLimit(taskHandle, channel, data)
    ccall((:DAQmxSetDOOvercurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetDOOvercurrentLimit(taskHandle, channel)
    ccall((:DAQmxResetDOOvercurrentLimit, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOOvercurrentAutoReenable(taskHandle, channel, data)
    ccall((:DAQmxGetDOOvercurrentAutoReenable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDOOvercurrentAutoReenable(taskHandle, channel, data)
    ccall((:DAQmxSetDOOvercurrentAutoReenable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDOOvercurrentAutoReenable(taskHandle, channel)
    ccall((:DAQmxResetDOOvercurrentAutoReenable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOOvercurrentReenablePeriod(taskHandle, channel, data)
    ccall((:DAQmxGetDOOvercurrentReenablePeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetDOOvercurrentReenablePeriod(taskHandle, channel, data)
    ccall((:DAQmxSetDOOvercurrentReenablePeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetDOOvercurrentReenablePeriod(taskHandle, channel)
    ccall((:DAQmxResetDOOvercurrentReenablePeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxGetDOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxSetDOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDOUseOnlyOnBrdMem(taskHandle, channel)
    ccall((:DAQmxResetDOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetDODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetDODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDODataXferMech(taskHandle, channel)
    ccall((:DAQmxResetDODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetDODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetDODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDODataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetDODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetDOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetDOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetDOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetDOUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetDOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetDOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetDOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetDOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetDOUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetDOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetDOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetDOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetDOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetDOMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetDOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDOGenerateOn(taskHandle, channel, data)
    ccall((:DAQmxGetDOGenerateOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetDOGenerateOn(taskHandle, channel, data)
    ccall((:DAQmxSetDOGenerateOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetDOGenerateOn(taskHandle, channel)
    ccall((:DAQmxResetDOGenerateOn, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIMax(taskHandle, channel, data)
    ccall((:DAQmxGetCIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIMax(taskHandle, channel, data)
    ccall((:DAQmxSetCIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIMax(taskHandle, channel)
    ccall((:DAQmxResetCIMax, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIMin(taskHandle, channel, data)
    ccall((:DAQmxGetCIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIMin(taskHandle, channel, data)
    ccall((:DAQmxSetCIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIMin(taskHandle, channel)
    ccall((:DAQmxResetCIMin, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICustomScaleName(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICustomScaleName(taskHandle, channel, data)
    ccall((:DAQmxSetCICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICustomScaleName(taskHandle, channel)
    ccall((:DAQmxResetCICustomScaleName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIMeasType(taskHandle, channel, data)
    ccall((:DAQmxGetCIMeasType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetCIFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFreqUnits(taskHandle, channel)
    ccall((:DAQmxResetCIFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIFreqTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIFreqTerm(taskHandle, channel)
    ccall((:DAQmxResetCIFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFreqTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFreqLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFreqThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFreqThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCIFreqThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFreqHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFreqHyst(taskHandle, channel)
    ccall((:DAQmxResetCIFreqHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIFreqDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFreqDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFreqDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIFreqDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIFreqDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFreqDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFreqDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIFreqDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFreqStartingEdge(taskHandle, channel)
    ccall((:DAQmxResetCIFreqStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqMeasMeth(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqMeasMeth(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFreqMeasMeth(taskHandle, channel)
    ccall((:DAQmxResetCIFreqMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqEnableAveraging(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqEnableAveraging(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIFreqEnableAveraging(taskHandle, channel)
    ccall((:DAQmxResetCIFreqEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqMeasTime(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFreqMeasTime(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFreqMeasTime(taskHandle, channel)
    ccall((:DAQmxResetCIFreqMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFreqDiv(taskHandle, channel, data)
    ccall((:DAQmxGetCIFreqDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIFreqDiv(taskHandle, channel, data)
    ccall((:DAQmxSetCIFreqDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIFreqDiv(taskHandle, channel)
    ccall((:DAQmxResetCIFreqDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodUnits(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPeriodTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPeriodTerm(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPeriodThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPeriodHyst(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPeriodDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodStartingEdge(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodMeasMeth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodMeasMeth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodMeasMeth(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodMeasMeth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodEnableAveraging(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodEnableAveraging(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodEnableAveraging(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodEnableAveraging, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodMeasTime(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodMeasTime(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPeriodMeasTime(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPeriodDiv(taskHandle, channel, data)
    ccall((:DAQmxGetCIPeriodDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIPeriodDiv(taskHandle, channel, data)
    ccall((:DAQmxSetCIPeriodDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIPeriodDiv(taskHandle, channel)
    ccall((:DAQmxResetCIPeriodDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesTerm(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesHyst(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDir(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesDir, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesDir(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDir, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDir(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDir, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesDirTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesDirTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesDirTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesDirTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesDirTerm(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesDirTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirHyst(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesCountDirDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesCountDirDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountDirDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountDirDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountDirDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountDirDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountDirDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountDirDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesInitialCnt(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesInitialCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesInitialCnt(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesInitialCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesInitialCnt(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesInitialCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesActiveEdge(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetResetCount(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetResetCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetResetCount(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetResetCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetResetCount(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetResetCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesCountResetTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesCountResetTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetTerm(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetHyst(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesCountResetDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesCountResetDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesCountResetActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesCountResetActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesCountResetActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesCountResetActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesCountResetActiveEdge(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesCountResetActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesGateTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesGateTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateTerm(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateHyst(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateHyst(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateHyst(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateHyst, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICountEdgesGateDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICountEdgesGateDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICountEdgesGateWhen(taskHandle, channel, data)
    ccall((:DAQmxGetCICountEdgesGateWhen, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICountEdgesGateWhen(taskHandle, channel, data)
    ccall((:DAQmxSetCICountEdgesGateWhen, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICountEdgesGateWhen(taskHandle, channel)
    ccall((:DAQmxResetCICountEdgesGateWhen, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIDutyCycleTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIDutyCycleTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleTerm(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIDutyCycleDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIDutyCycleDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDutyCycleStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIDutyCycleStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIDutyCycleStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIDutyCycleStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIDutyCycleStartingEdge(taskHandle, channel)
    ccall((:DAQmxResetCIDutyCycleStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIAngEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIAngEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIAngEncoderUnits(taskHandle, channel)
    ccall((:DAQmxResetCIAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIAngEncoderPulsesPerRev(taskHandle, channel, data)
    ccall((:DAQmxGetCIAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIAngEncoderPulsesPerRev(taskHandle, channel, data)
    ccall((:DAQmxSetCIAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIAngEncoderPulsesPerRev(taskHandle, channel)
    ccall((:DAQmxResetCIAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIAngEncoderInitialAngle(taskHandle, channel, data)
    ccall((:DAQmxGetCIAngEncoderInitialAngle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIAngEncoderInitialAngle(taskHandle, channel, data)
    ccall((:DAQmxSetCIAngEncoderInitialAngle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIAngEncoderInitialAngle(taskHandle, channel)
    ccall((:DAQmxResetCIAngEncoderInitialAngle, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCILinEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCILinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCILinEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCILinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCILinEncoderUnits(taskHandle, channel)
    ccall((:DAQmxResetCILinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCILinEncoderDistPerPulse(taskHandle, channel, data)
    ccall((:DAQmxGetCILinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCILinEncoderDistPerPulse(taskHandle, channel, data)
    ccall((:DAQmxSetCILinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCILinEncoderDistPerPulse(taskHandle, channel)
    ccall((:DAQmxResetCILinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCILinEncoderInitialPos(taskHandle, channel, data)
    ccall((:DAQmxGetCILinEncoderInitialPos, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCILinEncoderInitialPos(taskHandle, channel, data)
    ccall((:DAQmxSetCILinEncoderInitialPos, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCILinEncoderInitialPos(taskHandle, channel)
    ccall((:DAQmxResetCILinEncoderInitialPos, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderDecodingType(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderDecodingType(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderDecodingType(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderAInputTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputTerm(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderAInputDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderAInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderAInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderAInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderAInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderAInputDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderAInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderBInputTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputTerm(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderBInputDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderBInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderBInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderBInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderBInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderBInputDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderBInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderZInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderZInputTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputTerm(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIEncoderZInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIEncoderZInputDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZInputDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZInputDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZInputDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZIndexEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZIndexEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZIndexEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZIndexEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZIndexEnable(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZIndexEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZIndexVal(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZIndexVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZIndexVal(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZIndexVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZIndexVal(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZIndexVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIEncoderZIndexPhase(taskHandle, channel, data)
    ccall((:DAQmxGetCIEncoderZIndexPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIEncoderZIndexPhase(taskHandle, channel, data)
    ccall((:DAQmxSetCIEncoderZIndexPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIEncoderZIndexPhase(taskHandle, channel)
    ccall((:DAQmxResetCIEncoderZIndexPhase, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthUnits(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseWidthTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseWidthTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthTerm(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseWidthDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseWidthDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseWidthStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseWidthStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseWidthStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseWidthStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseWidthStartingEdge(taskHandle, channel)
    ccall((:DAQmxResetCIPulseWidthStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITimestampUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCITimestampUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITimestampUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCITimestampUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITimestampUnits(taskHandle, channel)
    ccall((:DAQmxResetCITimestampUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITimestampInitialSeconds(taskHandle, channel, data)
    ccall((:DAQmxGetCITimestampInitialSeconds, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCITimestampInitialSeconds(taskHandle, channel, data)
    ccall((:DAQmxSetCITimestampInitialSeconds, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCITimestampInitialSeconds(taskHandle, channel)
    ccall((:DAQmxResetCITimestampInitialSeconds, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIGPSSyncMethod(taskHandle, channel, data)
    ccall((:DAQmxGetCIGPSSyncMethod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIGPSSyncMethod(taskHandle, channel, data)
    ccall((:DAQmxSetCIGPSSyncMethod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIGPSSyncMethod(taskHandle, channel)
    ccall((:DAQmxResetCIGPSSyncMethod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIGPSSyncSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIGPSSyncSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIGPSSyncSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIGPSSyncSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIGPSSyncSrc(taskHandle, channel)
    ccall((:DAQmxResetCIGPSSyncSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityAngEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityAngEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityAngEncoderUnits(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityAngEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityAngEncoderPulsesPerRev(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityAngEncoderPulsesPerRev(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityAngEncoderPulsesPerRev(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityAngEncoderPulsesPerRev, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityLinEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityLinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityLinEncoderUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityLinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityLinEncoderUnits(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityLinEncoderUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityLinEncoderDistPerPulse(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityLinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityLinEncoderDistPerPulse(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityLinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityLinEncoderDistPerPulse(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityLinEncoderDistPerPulse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderDecodingType(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderDecodingType(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderDecodingType(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderDecodingType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIVelocityEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIVelocityEncoderAInputTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputTerm(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderAInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderAInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderAInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderAInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIVelocityEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIVelocityEncoderAInputDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderAInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderAInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderAInputDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderAInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIVelocityEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIVelocityEncoderBInputTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputTerm(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderBInputTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderBInputLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderBInputDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderBInputDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIVelocityEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIVelocityEncoderBInputDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityEncoderBInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityEncoderBInputDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityEncoderBInputDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityEncoderBInputDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityMeasTime(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityMeasTime(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIVelocityMeasTime(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityMeasTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIVelocityDiv(taskHandle, channel, data)
    ccall((:DAQmxGetCIVelocityDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIVelocityDiv(taskHandle, channel, data)
    ccall((:DAQmxSetCIVelocityDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIVelocityDiv(taskHandle, channel)
    ccall((:DAQmxResetCIVelocityDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepUnits(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCITwoEdgeSepFirstTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCITwoEdgeSepFirstTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstTerm(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepFirstEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepFirstEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepFirstEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepFirstEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepFirstEdge(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepFirstEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCITwoEdgeSepSecondTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCITwoEdgeSepSecondTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondTerm(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCITwoEdgeSepSecondEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCITwoEdgeSepSecondEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCITwoEdgeSepSecondEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCITwoEdgeSepSecondEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCITwoEdgeSepSecondEdge(taskHandle, channel)
    ccall((:DAQmxResetCITwoEdgeSepSecondEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodUnits(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCISemiPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCISemiPeriodTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodTerm(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCISemiPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCISemiPeriodDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISemiPeriodStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCISemiPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISemiPeriodStartingEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCISemiPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISemiPeriodStartingEdge(taskHandle, channel)
    ccall((:DAQmxResetCISemiPeriodStartingEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqUnits(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseFreqTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqTerm(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseFreqDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseFreqStartEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseFreqStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseFreqStartEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseFreqStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseFreqStartEdge(taskHandle, channel)
    ccall((:DAQmxResetCIPulseFreqStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeUnits(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseTimeTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseTimeTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeTerm(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseTimeDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseTimeDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTimeStartEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTimeStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTimeStartEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTimeStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTimeStartEdge(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTimeStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseTicksTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseTicksTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksTerm(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksTermCfg(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksTermCfg(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksTermCfg(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksTermCfg, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksLogicLvlBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksLogicLvlBehavior(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksLogicLvlBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCIPulseTicksDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCIPulseTicksDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPulseTicksStartEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCIPulseTicksStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIPulseTicksStartEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCIPulseTicksStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIPulseTicksStartEdge(taskHandle, channel)
    ccall((:DAQmxResetCIPulseTicksStartEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICtrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseActiveEdge(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCICtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCICtrTimebaseDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICtrTimebaseDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxGetCIThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIThreshVoltage(taskHandle, channel, data)
    ccall((:DAQmxSetCIThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIThreshVoltage(taskHandle, channel)
    ccall((:DAQmxResetCIThreshVoltage, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFilterEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIFilterEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIFilterEnable(taskHandle, channel)
    ccall((:DAQmxResetCIFilterEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFilterFreq(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIFilterFreq(taskHandle, channel, data)
    ccall((:DAQmxSetCIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIFilterFreq(taskHandle, channel)
    ccall((:DAQmxResetCIFilterFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFilterResponse(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFilterResponse(taskHandle, channel, data)
    ccall((:DAQmxSetCIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFilterResponse(taskHandle, channel)
    ccall((:DAQmxResetCIFilterResponse, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFilterOrder(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIFilterOrder(taskHandle, channel, data)
    ccall((:DAQmxSetCIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIFilterOrder(taskHandle, channel)
    ccall((:DAQmxResetCIFilterOrder, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIFilterDelay(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxGetCIFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIFilterDelayUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIFilterDelayUnits(taskHandle, channel)
    ccall((:DAQmxResetCIFilterDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCICount(taskHandle, channel, data)
    ccall((:DAQmxGetCICount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetCIOutputState(taskHandle, channel, data)
    ccall((:DAQmxGetCIOutputState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetCITCReached(taskHandle, channel, data)
    ccall((:DAQmxGetCITCReached, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetCICtrTimebaseMasterTimebaseDiv(taskHandle, channel, data)
    ccall((:DAQmxGetCICtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCICtrTimebaseMasterTimebaseDiv(taskHandle, channel, data)
    ccall((:DAQmxSetCICtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCICtrTimebaseMasterTimebaseDiv(taskHandle, channel)
    ccall((:DAQmxResetCICtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISampClkOverrunBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetCISampClkOverrunBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISampClkOverrunBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetCISampClkOverrunBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISampClkOverrunBehavior(taskHandle, channel)
    ccall((:DAQmxResetCISampClkOverrunBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCISampClkOverrunSentinelVal(taskHandle, channel, data)
    ccall((:DAQmxGetCISampClkOverrunSentinelVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCISampClkOverrunSentinelVal(taskHandle, channel, data)
    ccall((:DAQmxSetCISampClkOverrunSentinelVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCISampClkOverrunSentinelVal(taskHandle, channel)
    ccall((:DAQmxResetCISampClkOverrunSentinelVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetCIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIDataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetCIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIDataXferMech(taskHandle, channel)
    ccall((:DAQmxResetCIDataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetCIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCIDataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetCIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCIDataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetCIDataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetCIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetCIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetCIUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetCIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetCIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetCIUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetCIMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCINumPossiblyInvalidSamps(taskHandle, channel, data)
    ccall((:DAQmxGetCINumPossiblyInvalidSamps, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetCIDupCountPrevent(taskHandle, channel, data)
    ccall((:DAQmxGetCIDupCountPrevent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCIDupCountPrevent(taskHandle, channel, data)
    ccall((:DAQmxSetCIDupCountPrevent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCIDupCountPrevent(taskHandle, channel)
    ccall((:DAQmxResetCIDupCountPrevent, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIPrescaler(taskHandle, channel, data)
    ccall((:DAQmxGetCIPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCIPrescaler(taskHandle, channel, data)
    ccall((:DAQmxSetCIPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCIPrescaler(taskHandle, channel)
    ccall((:DAQmxResetCIPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCIMaxMeasPeriod(taskHandle, channel, data)
    ccall((:DAQmxGetCIMaxMeasPeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCIMaxMeasPeriod(taskHandle, channel, data)
    ccall((:DAQmxSetCIMaxMeasPeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCIMaxMeasPeriod(taskHandle, channel)
    ccall((:DAQmxResetCIMaxMeasPeriod, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOOutputType(taskHandle, channel, data)
    ccall((:DAQmxGetCOOutputType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetCOPulseIdleState(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseIdleState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseIdleState(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseIdleState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCOPulseIdleState(taskHandle, channel)
    ccall((:DAQmxResetCOPulseIdleState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseTerm(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCOPulseTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCOPulseTerm(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCOPulseTerm(taskHandle, channel)
    ccall((:DAQmxResetCOPulseTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseTimeUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseTimeUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCOPulseTimeUnits(taskHandle, channel)
    ccall((:DAQmxResetCOPulseTimeUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseHighTime(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseHighTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseHighTime(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseHighTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseHighTime(taskHandle, channel)
    ccall((:DAQmxResetCOPulseHighTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseLowTime(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseLowTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseLowTime(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseLowTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseLowTime(taskHandle, channel)
    ccall((:DAQmxResetCOPulseLowTime, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseTimeInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseTimeInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseTimeInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseTimeInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseTimeInitialDelay(taskHandle, channel)
    ccall((:DAQmxResetCOPulseTimeInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseDutyCyc(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseDutyCyc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseDutyCyc(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseDutyCyc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseDutyCyc(taskHandle, channel)
    ccall((:DAQmxResetCOPulseDutyCyc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseFreqUnits(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCOPulseFreqUnits(taskHandle, channel)
    ccall((:DAQmxResetCOPulseFreqUnits, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseFreq(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseFreq(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseFreq(taskHandle, channel)
    ccall((:DAQmxResetCOPulseFreq, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseFreqInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseFreqInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOPulseFreqInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseFreqInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOPulseFreqInitialDelay(taskHandle, channel)
    ccall((:DAQmxResetCOPulseFreqInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseHighTicks(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseHighTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseHighTicks(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseHighTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOPulseHighTicks(taskHandle, channel)
    ccall((:DAQmxResetCOPulseHighTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseLowTicks(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseLowTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseLowTicks(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseLowTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOPulseLowTicks(taskHandle, channel)
    ccall((:DAQmxResetCOPulseLowTicks, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseTicksInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseTicksInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOPulseTicksInitialDelay(taskHandle, channel, data)
    ccall((:DAQmxSetCOPulseTicksInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOPulseTicksInitialDelay(taskHandle, channel)
    ccall((:DAQmxResetCOPulseTicksInitialDelay, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCOCtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCOCtrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseActiveEdge(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseActiveEdge(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseDigFltrEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseDigFltrEnable(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseDigFltrMinPulseWidth(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseDigFltrMinPulseWidth(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseDigFltrTimebaseSrc(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetCOCtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetCOCtrTimebaseDigFltrTimebaseSrc(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseDigFltrTimebaseSrc(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseDigFltrTimebaseRate(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseDigFltrTimebaseRate(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseDigSyncEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseDigSyncEnable(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCount(taskHandle, channel, data)
    ccall((:DAQmxGetCOCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxGetCOOutputState(taskHandle, channel, data)
    ccall((:DAQmxGetCOOutputState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetCOAutoIncrCnt(taskHandle, channel, data)
    ccall((:DAQmxGetCOAutoIncrCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOAutoIncrCnt(taskHandle, channel, data)
    ccall((:DAQmxSetCOAutoIncrCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOAutoIncrCnt(taskHandle, channel)
    ccall((:DAQmxResetCOAutoIncrCnt, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOCtrTimebaseMasterTimebaseDiv(taskHandle, channel, data)
    ccall((:DAQmxGetCOCtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOCtrTimebaseMasterTimebaseDiv(taskHandle, channel, data)
    ccall((:DAQmxSetCOCtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOCtrTimebaseMasterTimebaseDiv(taskHandle, channel)
    ccall((:DAQmxResetCOCtrTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPulseDone(taskHandle, channel, data)
    ccall((:DAQmxGetCOPulseDone, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetCOEnableInitialDelayOnRetrigger(taskHandle, channel, data)
    ccall((:DAQmxGetCOEnableInitialDelayOnRetrigger, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCOEnableInitialDelayOnRetrigger(taskHandle, channel, data)
    ccall((:DAQmxSetCOEnableInitialDelayOnRetrigger, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCOEnableInitialDelayOnRetrigger(taskHandle, channel)
    ccall((:DAQmxResetCOEnableInitialDelayOnRetrigger, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOConstrainedGenMode(taskHandle, channel, data)
    ccall((:DAQmxGetCOConstrainedGenMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCOConstrainedGenMode(taskHandle, channel, data)
    ccall((:DAQmxSetCOConstrainedGenMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCOConstrainedGenMode(taskHandle, channel)
    ccall((:DAQmxResetCOConstrainedGenMode, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxGetCOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCOUseOnlyOnBrdMem(taskHandle, channel, data)
    ccall((:DAQmxSetCOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCOUseOnlyOnBrdMem(taskHandle, channel)
    ccall((:DAQmxResetCOUseOnlyOnBrdMem, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxGetCODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCODataXferMech(taskHandle, channel, data)
    ccall((:DAQmxSetCODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCODataXferMech(taskHandle, channel)
    ccall((:DAQmxResetCODataXferMech, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxGetCODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetCODataXferReqCond(taskHandle, channel, data)
    ccall((:DAQmxSetCODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetCODataXferReqCond(taskHandle, channel)
    ccall((:DAQmxResetCODataXferReqCond, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxGetCOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOUsbXferReqSize(taskHandle, channel, data)
    ccall((:DAQmxSetCOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOUsbXferReqSize(taskHandle, channel)
    ccall((:DAQmxResetCOUsbXferReqSize, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxGetCOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOUsbXferReqCount(taskHandle, channel, data)
    ccall((:DAQmxSetCOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOUsbXferReqCount(taskHandle, channel)
    ccall((:DAQmxResetCOUsbXferReqCount, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxGetCOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetCOMemMapEnable(taskHandle, channel, data)
    ccall((:DAQmxSetCOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetCOMemMapEnable(taskHandle, channel)
    ccall((:DAQmxResetCOMemMapEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCOPrescaler(taskHandle, channel, data)
    ccall((:DAQmxGetCOPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, channel, data)
end

function DAQmxSetCOPrescaler(taskHandle, channel, data)
    ccall((:DAQmxSetCOPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, channel, data)
end

function DAQmxResetCOPrescaler(taskHandle, channel)
    ccall((:DAQmxResetCOPrescaler, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetCORdyForNewVal(taskHandle, channel, data)
    ccall((:DAQmxGetCORdyForNewVal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetPwrVoltageSetpoint(taskHandle, channel, data)
    ccall((:DAQmxGetPwrVoltageSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetPwrVoltageSetpoint(taskHandle, channel, data)
    ccall((:DAQmxSetPwrVoltageSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetPwrVoltageSetpoint(taskHandle, channel)
    ccall((:DAQmxResetPwrVoltageSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetPwrVoltageDevScalingCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetPwrVoltageDevScalingCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxGetPwrCurrentSetpoint(taskHandle, channel, data)
    ccall((:DAQmxGetPwrCurrentSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, channel, data)
end

function DAQmxSetPwrCurrentSetpoint(taskHandle, channel, data)
    ccall((:DAQmxSetPwrCurrentSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, channel, data)
end

function DAQmxResetPwrCurrentSetpoint(taskHandle, channel)
    ccall((:DAQmxResetPwrCurrentSetpoint, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetPwrCurrentDevScalingCoeff(taskHandle, channel, data, arraySizeInElements)
    ccall((:DAQmxGetPwrCurrentDevScalingCoeff, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}, uInt32), taskHandle, channel, data, arraySizeInElements)
end

function DAQmxGetPwrOutputEnable(taskHandle, channel, data)
    ccall((:DAQmxGetPwrOutputEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxSetPwrOutputEnable(taskHandle, channel, data)
    ccall((:DAQmxSetPwrOutputEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, channel, data)
end

function DAQmxResetPwrOutputEnable(taskHandle, channel)
    ccall((:DAQmxResetPwrOutputEnable, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetPwrOutputState(taskHandle, channel, data)
    ccall((:DAQmxGetPwrOutputState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetPwrIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetPwrIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetPwrIdleOutputBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetPwrIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetPwrIdleOutputBehavior(taskHandle, channel)
    ccall((:DAQmxResetPwrIdleOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetPwrRemoteSense(taskHandle, channel, data)
    ccall((:DAQmxGetPwrRemoteSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetPwrRemoteSense(taskHandle, channel, data)
    ccall((:DAQmxSetPwrRemoteSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetPwrRemoteSense(taskHandle, channel)
    ccall((:DAQmxResetPwrRemoteSense, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetChanType(taskHandle, channel, data)
    ccall((:DAQmxGetChanType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxGetPhysicalChanName(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetPhysicalChanName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetPhysicalChanName(taskHandle, channel, data)
    ccall((:DAQmxSetPhysicalChanName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxGetChanDescr(taskHandle, channel, data, bufferSize)
    ccall((:DAQmxGetChanDescr, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, channel, data, bufferSize)
end

function DAQmxSetChanDescr(taskHandle, channel, data)
    ccall((:DAQmxSetChanDescr, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, channel, data)
end

function DAQmxResetChanDescr(taskHandle, channel)
    ccall((:DAQmxResetChanDescr, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetChanIsGlobal(taskHandle, channel, data)
    ccall((:DAQmxGetChanIsGlobal, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, channel, data)
end

function DAQmxGetChanSyncUnlockBehavior(taskHandle, channel, data)
    ccall((:DAQmxGetChanSyncUnlockBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, channel, data)
end

function DAQmxSetChanSyncUnlockBehavior(taskHandle, channel, data)
    ccall((:DAQmxSetChanSyncUnlockBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, channel, data)
end

function DAQmxResetChanSyncUnlockBehavior(taskHandle, channel)
    ccall((:DAQmxResetChanSyncUnlockBehavior, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, channel)
end

function DAQmxGetDevIsSimulated(device, data)
    ccall((:DAQmxGetDevIsSimulated, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevProductCategory(device, data)
    ccall((:DAQmxGetDevProductCategory, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevProductType(device, data, bufferSize)
    ccall((:DAQmxGetDevProductType, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevProductNum(device, data)
    ccall((:DAQmxGetDevProductNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevSerialNum(device, data)
    ccall((:DAQmxGetDevSerialNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevAccessoryProductTypes(device, data, bufferSize)
    ccall((:DAQmxGetDevAccessoryProductTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevAccessoryProductNums(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAccessoryProductNums, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAccessorySerialNums(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAccessorySerialNums, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetCarrierSerialNum(device, data)
    ccall((:DAQmxGetCarrierSerialNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetFieldDAQDevName(device, data, bufferSize)
    ccall((:DAQmxGetFieldDAQDevName, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetFieldDAQBankDevNames(device, data, bufferSize)
    ccall((:DAQmxGetFieldDAQBankDevNames, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevChassisModuleDevNames(device, data, bufferSize)
    ccall((:DAQmxGetDevChassisModuleDevNames, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevAnlgTrigSupported(device, data)
    ccall((:DAQmxGetDevAnlgTrigSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevDigTrigSupported(device, data)
    ccall((:DAQmxGetDevDigTrigSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevTimeTrigSupported(device, data)
    ccall((:DAQmxGetDevTimeTrigSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevAIPhysicalChans(device, data, bufferSize)
    ccall((:DAQmxGetDevAIPhysicalChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevAISupportedMeasTypes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAISupportedMeasTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIMaxSingleChanRate(device, data)
    ccall((:DAQmxGetDevAIMaxSingleChanRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevAIMaxMultiChanRate(device, data)
    ccall((:DAQmxGetDevAIMaxMultiChanRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevAIMinRate(device, data)
    ccall((:DAQmxGetDevAIMinRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevAISimultaneousSamplingSupported(device, data)
    ccall((:DAQmxGetDevAISimultaneousSamplingSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevAINumSampTimingEngines(device, data)
    ccall((:DAQmxGetDevAINumSampTimingEngines, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevAISampModes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAISampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAINumSyncPulseSrcs(device, data)
    ccall((:DAQmxGetDevAINumSyncPulseSrcs, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevAITrigUsage(device, data)
    ccall((:DAQmxGetDevAITrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevAIVoltageRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIVoltageRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIVoltageIntExcitDiscreteVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIVoltageIntExcitDiscreteVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIVoltageIntExcitRangeVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIVoltageIntExcitRangeVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIChargeRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIChargeRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAICurrentRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAICurrentRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAICurrentIntExcitDiscreteVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAICurrentIntExcitDiscreteVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIBridgeRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIBridgeRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIResistanceRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIResistanceRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIFreqRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIFreqRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIGains(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIGains, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAICouplings(device, data)
    ccall((:DAQmxGetDevAICouplings, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevAILowpassCutoffFreqDiscreteVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAILowpassCutoffFreqDiscreteVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAILowpassCutoffFreqRangeVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAILowpassCutoffFreqRangeVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetAIDigFltrTypes(device, data, arraySizeInElements)
    ccall((:DAQmxGetAIDigFltrTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIDigFltrLowpassCutoffFreqDiscreteVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIDigFltrLowpassCutoffFreqDiscreteVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAIDigFltrLowpassCutoffFreqRangeVals(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAIDigFltrLowpassCutoffFreqRangeVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAOPhysicalChans(device, data, bufferSize)
    ccall((:DAQmxGetDevAOPhysicalChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevAOSupportedOutputTypes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAOSupportedOutputTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAOMaxRate(device, data)
    ccall((:DAQmxGetDevAOMaxRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevAOMinRate(device, data)
    ccall((:DAQmxGetDevAOMinRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevAOSampClkSupported(device, data)
    ccall((:DAQmxGetDevAOSampClkSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevAONumSampTimingEngines(device, data)
    ccall((:DAQmxGetDevAONumSampTimingEngines, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevAOSampModes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAOSampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAONumSyncPulseSrcs(device, data)
    ccall((:DAQmxGetDevAONumSyncPulseSrcs, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevAOTrigUsage(device, data)
    ccall((:DAQmxGetDevAOTrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevAOVoltageRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAOVoltageRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAOCurrentRngs(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAOCurrentRngs, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevAOGains(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevAOGains, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevDILines(device, data, bufferSize)
    ccall((:DAQmxGetDevDILines, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevDIPorts(device, data, bufferSize)
    ccall((:DAQmxGetDevDIPorts, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevDIMaxRate(device, data)
    ccall((:DAQmxGetDevDIMaxRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevDINumSampTimingEngines(device, data)
    ccall((:DAQmxGetDevDINumSampTimingEngines, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevDITrigUsage(device, data)
    ccall((:DAQmxGetDevDITrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevDOLines(device, data, bufferSize)
    ccall((:DAQmxGetDevDOLines, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevDOPorts(device, data, bufferSize)
    ccall((:DAQmxGetDevDOPorts, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevDOMaxRate(device, data)
    ccall((:DAQmxGetDevDOMaxRate, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevDONumSampTimingEngines(device, data)
    ccall((:DAQmxGetDevDONumSampTimingEngines, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevDOTrigUsage(device, data)
    ccall((:DAQmxGetDevDOTrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevCIPhysicalChans(device, data, bufferSize)
    ccall((:DAQmxGetDevCIPhysicalChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevCISupportedMeasTypes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevCISupportedMeasTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevCITrigUsage(device, data)
    ccall((:DAQmxGetDevCITrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevCISampClkSupported(device, data)
    ccall((:DAQmxGetDevCISampClkSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevCISampModes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevCISampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevCIMaxSize(device, data)
    ccall((:DAQmxGetDevCIMaxSize, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevCIMaxTimebase(device, data)
    ccall((:DAQmxGetDevCIMaxTimebase, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevCOPhysicalChans(device, data, bufferSize)
    ccall((:DAQmxGetDevCOPhysicalChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevCOSupportedOutputTypes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevCOSupportedOutputTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevCOSampClkSupported(device, data)
    ccall((:DAQmxGetDevCOSampClkSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevCOSampModes(device, data, arraySizeInElements)
    ccall((:DAQmxGetDevCOSampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), device, data, arraySizeInElements)
end

function DAQmxGetDevCOTrigUsage(device, data)
    ccall((:DAQmxGetDevCOTrigUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevCOMaxSize(device, data)
    ccall((:DAQmxGetDevCOMaxSize, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevCOMaxTimebase(device, data)
    ccall((:DAQmxGetDevCOMaxTimebase, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), device, data)
end

function DAQmxGetDevTEDSHWTEDSSupported(device, data)
    ccall((:DAQmxGetDevTEDSHWTEDSSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), device, data)
end

function DAQmxGetDevNumDMAChans(device, data)
    ccall((:DAQmxGetDevNumDMAChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevBusType(device, data)
    ccall((:DAQmxGetDevBusType, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), device, data)
end

function DAQmxGetDevPCIBusNum(device, data)
    ccall((:DAQmxGetDevPCIBusNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevPCIDevNum(device, data)
    ccall((:DAQmxGetDevPCIDevNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevPXIChassisNum(device, data)
    ccall((:DAQmxGetDevPXIChassisNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevPXISlotNum(device, data)
    ccall((:DAQmxGetDevPXISlotNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevCompactDAQChassisDevName(device, data, bufferSize)
    ccall((:DAQmxGetDevCompactDAQChassisDevName, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevCompactDAQSlotNum(device, data)
    ccall((:DAQmxGetDevCompactDAQSlotNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevCompactRIOChassisDevName(device, data, bufferSize)
    ccall((:DAQmxGetDevCompactRIOChassisDevName, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevCompactRIOSlotNum(device, data)
    ccall((:DAQmxGetDevCompactRIOSlotNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevTCPIPHostname(device, data, bufferSize)
    ccall((:DAQmxGetDevTCPIPHostname, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevTCPIPEthernetIP(device, data, bufferSize)
    ccall((:DAQmxGetDevTCPIPEthernetIP, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevTCPIPWirelessIP(device, data, bufferSize)
    ccall((:DAQmxGetDevTCPIPWirelessIP, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevTerminals(device, data, bufferSize)
    ccall((:DAQmxGetDevTerminals, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), device, data, bufferSize)
end

function DAQmxGetDevNumTimeTrigs(device, data)
    ccall((:DAQmxGetDevNumTimeTrigs, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetDevNumTimestampEngines(device, data)
    ccall((:DAQmxGetDevNumTimestampEngines, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), device, data)
end

function DAQmxGetExportedAIConvClkOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedAIConvClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedAIConvClkOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedAIConvClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedAIConvClkOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedAIConvClkOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAIConvClkPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedAIConvClkPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxGetExported10MHzRefClkOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExported10MHzRefClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExported10MHzRefClkOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExported10MHzRefClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExported10MHzRefClkOutputTerm(taskHandle)
    ccall((:DAQmxResetExported10MHzRefClkOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExported20MHzTimebaseOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExported20MHzTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExported20MHzTimebaseOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExported20MHzTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExported20MHzTimebaseOutputTerm(taskHandle)
    ccall((:DAQmxResetExported20MHzTimebaseOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSampClkOutputBehavior(taskHandle, data)
    ccall((:DAQmxGetExportedSampClkOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedSampClkOutputBehavior(taskHandle, data)
    ccall((:DAQmxSetExportedSampClkOutputBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedSampClkOutputBehavior(taskHandle)
    ccall((:DAQmxResetExportedSampClkOutputBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSampClkOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedSampClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedSampClkOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedSampClkOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedSampClkOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedSampClkOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSampClkDelayOffset(taskHandle, data)
    ccall((:DAQmxGetExportedSampClkDelayOffset, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedSampClkDelayOffset(taskHandle, data)
    ccall((:DAQmxSetExportedSampClkDelayOffset, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedSampClkDelayOffset(taskHandle)
    ccall((:DAQmxResetExportedSampClkDelayOffset, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSampClkPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedSampClkPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedSampClkPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedSampClkPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedSampClkPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedSampClkPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSampClkTimebaseOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedSampClkTimebaseOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedSampClkTimebaseOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedDividedSampClkTimebaseOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedDividedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedDividedSampClkTimebaseOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedDividedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedDividedSampClkTimebaseOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedDividedSampClkTimebaseOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvTrigOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedAdvTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedAdvTrigOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedAdvTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedAdvTrigOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedAdvTrigOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvTrigPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedAdvTrigPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxGetExportedAdvTrigPulseWidthUnits(taskHandle, data)
    ccall((:DAQmxGetExportedAdvTrigPulseWidthUnits, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedAdvTrigPulseWidthUnits(taskHandle, data)
    ccall((:DAQmxSetExportedAdvTrigPulseWidthUnits, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedAdvTrigPulseWidthUnits(taskHandle)
    ccall((:DAQmxResetExportedAdvTrigPulseWidthUnits, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvTrigPulseWidth(taskHandle, data)
    ccall((:DAQmxGetExportedAdvTrigPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedAdvTrigPulseWidth(taskHandle, data)
    ccall((:DAQmxSetExportedAdvTrigPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedAdvTrigPulseWidth(taskHandle)
    ccall((:DAQmxResetExportedAdvTrigPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedPauseTrigOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedPauseTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedPauseTrigOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedPauseTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedPauseTrigOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedPauseTrigOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedPauseTrigLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxGetExportedPauseTrigLvlActiveLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedPauseTrigLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxSetExportedPauseTrigLvlActiveLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedPauseTrigLvlActiveLvl(taskHandle)
    ccall((:DAQmxResetExportedPauseTrigLvlActiveLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRefTrigOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedRefTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedRefTrigOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedRefTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedRefTrigOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedRefTrigOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRefTrigPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedRefTrigPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedRefTrigPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedRefTrigPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedRefTrigPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedRefTrigPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedStartTrigOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedStartTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedStartTrigOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedStartTrigOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedStartTrigOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedStartTrigOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedStartTrigPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedStartTrigPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedStartTrigPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedStartTrigPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedStartTrigPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedStartTrigPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvCmpltEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedAdvCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedAdvCmpltEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedAdvCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedAdvCmpltEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedAdvCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvCmpltEventDelay(taskHandle, data)
    ccall((:DAQmxGetExportedAdvCmpltEventDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedAdvCmpltEventDelay(taskHandle, data)
    ccall((:DAQmxSetExportedAdvCmpltEventDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedAdvCmpltEventDelay(taskHandle)
    ccall((:DAQmxResetExportedAdvCmpltEventDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvCmpltEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedAdvCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedAdvCmpltEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedAdvCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedAdvCmpltEventPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedAdvCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAdvCmpltEventPulseWidth(taskHandle, data)
    ccall((:DAQmxGetExportedAdvCmpltEventPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedAdvCmpltEventPulseWidth(taskHandle, data)
    ccall((:DAQmxSetExportedAdvCmpltEventPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedAdvCmpltEventPulseWidth(taskHandle)
    ccall((:DAQmxResetExportedAdvCmpltEventPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAIHoldCmpltEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedAIHoldCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedAIHoldCmpltEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedAIHoldCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedAIHoldCmpltEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedAIHoldCmpltEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedAIHoldCmpltEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedAIHoldCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedAIHoldCmpltEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedAIHoldCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedAIHoldCmpltEventPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedAIHoldCmpltEventPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedChangeDetectEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedChangeDetectEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedChangeDetectEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedChangeDetectEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedChangeDetectEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedChangeDetectEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedChangeDetectEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedChangeDetectEventPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedChangeDetectEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedChangeDetectEventPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedChangeDetectEventPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedChangeDetectEventPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedCtrOutEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedCtrOutEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedCtrOutEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedCtrOutEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedCtrOutEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedCtrOutEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedCtrOutEventOutputBehavior(taskHandle, data)
    ccall((:DAQmxGetExportedCtrOutEventOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedCtrOutEventOutputBehavior(taskHandle, data)
    ccall((:DAQmxSetExportedCtrOutEventOutputBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedCtrOutEventOutputBehavior(taskHandle)
    ccall((:DAQmxResetExportedCtrOutEventOutputBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedCtrOutEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedCtrOutEventPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedCtrOutEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedCtrOutEventPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedCtrOutEventPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedCtrOutEventPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedCtrOutEventToggleIdleState(taskHandle, data)
    ccall((:DAQmxGetExportedCtrOutEventToggleIdleState, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedCtrOutEventToggleIdleState(taskHandle, data)
    ccall((:DAQmxSetExportedCtrOutEventToggleIdleState, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedCtrOutEventToggleIdleState(taskHandle)
    ccall((:DAQmxResetExportedCtrOutEventToggleIdleState, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedHshkEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedHshkEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedHshkEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedHshkEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventOutputBehavior(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventOutputBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedHshkEventOutputBehavior(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventOutputBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedHshkEventOutputBehavior(taskHandle)
    ccall((:DAQmxResetExportedHshkEventOutputBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventDelay(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedHshkEventDelay(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedHshkEventDelay(taskHandle)
    ccall((:DAQmxResetExportedHshkEventDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventInterlockedAssertedLvl(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventInterlockedAssertedLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedHshkEventInterlockedAssertedLvl(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventInterlockedAssertedLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedHshkEventInterlockedAssertedLvl(taskHandle)
    ccall((:DAQmxResetExportedHshkEventInterlockedAssertedLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventInterlockedAssertOnStart(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventInterlockedAssertOnStart, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetExportedHshkEventInterlockedAssertOnStart(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventInterlockedAssertOnStart, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetExportedHshkEventInterlockedAssertOnStart(taskHandle)
    ccall((:DAQmxResetExportedHshkEventInterlockedAssertOnStart, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventInterlockedDeassertDelay(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventInterlockedDeassertDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedHshkEventInterlockedDeassertDelay(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventInterlockedDeassertDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedHshkEventInterlockedDeassertDelay(taskHandle)
    ccall((:DAQmxResetExportedHshkEventInterlockedDeassertDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventPulsePolarity, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedHshkEventPulsePolarity(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventPulsePolarity, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedHshkEventPulsePolarity(taskHandle)
    ccall((:DAQmxResetExportedHshkEventPulsePolarity, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedHshkEventPulseWidth(taskHandle, data)
    ccall((:DAQmxGetExportedHshkEventPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetExportedHshkEventPulseWidth(taskHandle, data)
    ccall((:DAQmxSetExportedHshkEventPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetExportedHshkEventPulseWidth(taskHandle)
    ccall((:DAQmxResetExportedHshkEventPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForXferEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedRdyForXferEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedRdyForXferEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForXferEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedRdyForXferEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedRdyForXferEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForXferEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxGetExportedRdyForXferEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedRdyForXferEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForXferEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedRdyForXferEventLvlActiveLvl(taskHandle)
    ccall((:DAQmxResetExportedRdyForXferEventLvlActiveLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForXferEventDeassertCond(taskHandle, data)
    ccall((:DAQmxGetExportedRdyForXferEventDeassertCond, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedRdyForXferEventDeassertCond(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForXferEventDeassertCond, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedRdyForXferEventDeassertCond(taskHandle)
    ccall((:DAQmxResetExportedRdyForXferEventDeassertCond, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForXferEventDeassertCondCustomThreshold(taskHandle, data)
    ccall((:DAQmxGetExportedRdyForXferEventDeassertCondCustomThreshold, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetExportedRdyForXferEventDeassertCondCustomThreshold(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForXferEventDeassertCondCustomThreshold, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetExportedRdyForXferEventDeassertCondCustomThreshold(taskHandle)
    ccall((:DAQmxResetExportedRdyForXferEventDeassertCondCustomThreshold, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedDataActiveEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedDataActiveEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedDataActiveEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedDataActiveEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedDataActiveEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedDataActiveEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedDataActiveEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxGetExportedDataActiveEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedDataActiveEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxSetExportedDataActiveEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedDataActiveEventLvlActiveLvl(taskHandle)
    ccall((:DAQmxResetExportedDataActiveEventLvlActiveLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForStartEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedRdyForStartEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedRdyForStartEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForStartEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedRdyForStartEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedRdyForStartEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedRdyForStartEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxGetExportedRdyForStartEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetExportedRdyForStartEventLvlActiveLvl(taskHandle, data)
    ccall((:DAQmxSetExportedRdyForStartEventLvlActiveLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetExportedRdyForStartEventLvlActiveLvl(taskHandle)
    ccall((:DAQmxResetExportedRdyForStartEventLvlActiveLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedSyncPulseEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedSyncPulseEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedSyncPulseEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedSyncPulseEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedSyncPulseEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedSyncPulseEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetExportedWatchdogExpiredEventOutputTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetExportedWatchdogExpiredEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetExportedWatchdogExpiredEventOutputTerm(taskHandle, data)
    ccall((:DAQmxSetExportedWatchdogExpiredEventOutputTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetExportedWatchdogExpiredEventOutputTerm(taskHandle)
    ccall((:DAQmxResetExportedWatchdogExpiredEventOutputTerm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetPersistedChanAuthor(channel, data, bufferSize)
    ccall((:DAQmxGetPersistedChanAuthor, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), channel, data, bufferSize)
end

function DAQmxGetPersistedChanAllowInteractiveEditing(channel, data)
    ccall((:DAQmxGetPersistedChanAllowInteractiveEditing, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), channel, data)
end

function DAQmxGetPersistedChanAllowInteractiveDeletion(channel, data)
    ccall((:DAQmxGetPersistedChanAllowInteractiveDeletion, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), channel, data)
end

function DAQmxGetPersistedScaleAuthor(scaleName, data, bufferSize)
    ccall((:DAQmxGetPersistedScaleAuthor, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), scaleName, data, bufferSize)
end

function DAQmxGetPersistedScaleAllowInteractiveEditing(scaleName, data)
    ccall((:DAQmxGetPersistedScaleAllowInteractiveEditing, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), scaleName, data)
end

function DAQmxGetPersistedScaleAllowInteractiveDeletion(scaleName, data)
    ccall((:DAQmxGetPersistedScaleAllowInteractiveDeletion, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), scaleName, data)
end

function DAQmxGetPersistedTaskAuthor(taskName, data, bufferSize)
    ccall((:DAQmxGetPersistedTaskAuthor, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), taskName, data, bufferSize)
end

function DAQmxGetPersistedTaskAllowInteractiveEditing(taskName, data)
    ccall((:DAQmxGetPersistedTaskAllowInteractiveEditing, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), taskName, data)
end

function DAQmxGetPersistedTaskAllowInteractiveDeletion(taskName, data)
    ccall((:DAQmxGetPersistedTaskAllowInteractiveDeletion, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), taskName, data)
end

function DAQmxGetPhysicalChanAISupportedMeasTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanAISupportedMeasTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanAITermCfgs(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAITermCfgs, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAIInputSrcs(physicalChannel, data, bufferSize)
    ccall((:DAQmxGetPhysicalChanAIInputSrcs, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), physicalChannel, data, bufferSize)
end

function DAQmxGetPhysicalChanAISensorPowerTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanAISensorPowerTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanAISensorPowerVoltageRangeVals(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanAISensorPowerVoltageRangeVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanAIPowerControlVoltage(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAIPowerControlVoltage, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), physicalChannel, data)
end

function DAQmxSetPhysicalChanAIPowerControlVoltage(physicalChannel, data)
    ccall((:DAQmxSetPhysicalChanAIPowerControlVoltage, nidaqmx), int32, (Ptr{Cchar}, float64), physicalChannel, data)
end

function DAQmxResetPhysicalChanAIPowerControlVoltage(physicalChannel)
    ccall((:DAQmxResetPhysicalChanAIPowerControlVoltage, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxGetPhysicalChanAIPowerControlEnable(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAIPowerControlEnable, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxSetPhysicalChanAIPowerControlEnable(physicalChannel, data)
    ccall((:DAQmxSetPhysicalChanAIPowerControlEnable, nidaqmx), int32, (Ptr{Cchar}, bool32), physicalChannel, data)
end

function DAQmxResetPhysicalChanAIPowerControlEnable(physicalChannel)
    ccall((:DAQmxResetPhysicalChanAIPowerControlEnable, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxGetPhysicalChanAIPowerControlType(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAIPowerControlType, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), physicalChannel, data)
end

function DAQmxSetPhysicalChanAIPowerControlType(physicalChannel, data)
    ccall((:DAQmxSetPhysicalChanAIPowerControlType, nidaqmx), int32, (Ptr{Cchar}, int32), physicalChannel, data)
end

function DAQmxResetPhysicalChanAIPowerControlType(physicalChannel)
    ccall((:DAQmxResetPhysicalChanAIPowerControlType, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxGetPhysicalChanAISensorPowerOpenChan(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAISensorPowerOpenChan, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAISensorPowerOvercurrent(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAISensorPowerOvercurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAOSupportedOutputTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanAOSupportedOutputTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanAOSupportedPowerUpOutputTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanAOSupportedPowerUpOutputTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanAOTermCfgs(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAOTermCfgs, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAOManualControlEnable(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAOManualControlEnable, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxSetPhysicalChanAOManualControlEnable(physicalChannel, data)
    ccall((:DAQmxSetPhysicalChanAOManualControlEnable, nidaqmx), int32, (Ptr{Cchar}, bool32), physicalChannel, data)
end

function DAQmxResetPhysicalChanAOManualControlEnable(physicalChannel)
    ccall((:DAQmxResetPhysicalChanAOManualControlEnable, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxGetPhysicalChanAOManualControlShortDetected(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAOManualControlShortDetected, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAOManualControlAmplitude(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAOManualControlAmplitude, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), physicalChannel, data)
end

function DAQmxGetPhysicalChanAOManualControlFreq(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanAOManualControlFreq, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), physicalChannel, data)
end

function DAQmxGetAOPowerAmpChannelEnable(physicalChannel, data)
    ccall((:DAQmxGetAOPowerAmpChannelEnable, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxSetAOPowerAmpChannelEnable(physicalChannel, data)
    ccall((:DAQmxSetAOPowerAmpChannelEnable, nidaqmx), int32, (Ptr{Cchar}, bool32), physicalChannel, data)
end

function DAQmxResetAOPowerAmpChannelEnable(physicalChannel)
    ccall((:DAQmxResetAOPowerAmpChannelEnable, nidaqmx), int32, (Ptr{Cchar},), physicalChannel)
end

function DAQmxGetAOPowerAmpScalingCoeff(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetAOPowerAmpScalingCoeff, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetAOPowerAmpOvercurrent(physicalChannel, data)
    ccall((:DAQmxGetAOPowerAmpOvercurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetAOPowerAmpGain(physicalChannel, data)
    ccall((:DAQmxGetAOPowerAmpGain, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), physicalChannel, data)
end

function DAQmxGetAOPowerAmpOffset(physicalChannel, data)
    ccall((:DAQmxGetAOPowerAmpOffset, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDIPortWidth(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanDIPortWidth, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDISampClkSupported(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanDISampClkSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDISampModes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanDISampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanDIChangeDetectSupported(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanDIChangeDetectSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDOPortWidth(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanDOPortWidth, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDOSampClkSupported(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanDOSampClkSupported, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanDOSampModes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanDOSampModes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanCISupportedMeasTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanCISupportedMeasTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanCOSupportedOutputTypes(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanCOSupportedOutputTypes, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanTEDSMfgID(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanTEDSMfgID, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanTEDSModelNum(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanTEDSModelNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanTEDSSerialNum(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanTEDSSerialNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanTEDSVersionNum(physicalChannel, data)
    ccall((:DAQmxGetPhysicalChanTEDSVersionNum, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), physicalChannel, data)
end

function DAQmxGetPhysicalChanTEDSVersionLetter(physicalChannel, data, bufferSize)
    ccall((:DAQmxGetPhysicalChanTEDSVersionLetter, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), physicalChannel, data, bufferSize)
end

function DAQmxGetPhysicalChanTEDSBitStream(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanTEDSBitStream, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt8}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetPhysicalChanTEDSTemplateIDs(physicalChannel, data, arraySizeInElements)
    ccall((:DAQmxGetPhysicalChanTEDSTemplateIDs, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}, uInt32), physicalChannel, data, arraySizeInElements)
end

function DAQmxGetReadRelativeTo(taskHandle, data)
    ccall((:DAQmxGetReadRelativeTo, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetReadRelativeTo(taskHandle, data)
    ccall((:DAQmxSetReadRelativeTo, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetReadRelativeTo(taskHandle)
    ccall((:DAQmxResetReadRelativeTo, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadOffset(taskHandle, data)
    ccall((:DAQmxGetReadOffset, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetReadOffset(taskHandle, data)
    ccall((:DAQmxSetReadOffset, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetReadOffset(taskHandle)
    ccall((:DAQmxResetReadOffset, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadChannelsToRead(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadChannelsToRead, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetReadChannelsToRead(taskHandle, data)
    ccall((:DAQmxSetReadChannelsToRead, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetReadChannelsToRead(taskHandle)
    ccall((:DAQmxResetReadChannelsToRead, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadReadAllAvailSamp(taskHandle, data)
    ccall((:DAQmxGetReadReadAllAvailSamp, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetReadReadAllAvailSamp(taskHandle, data)
    ccall((:DAQmxSetReadReadAllAvailSamp, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetReadReadAllAvailSamp(taskHandle)
    ccall((:DAQmxResetReadReadAllAvailSamp, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadAutoStart(taskHandle, data)
    ccall((:DAQmxGetReadAutoStart, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetReadAutoStart(taskHandle, data)
    ccall((:DAQmxSetReadAutoStart, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetReadAutoStart(taskHandle)
    ccall((:DAQmxResetReadAutoStart, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadOverWrite(taskHandle, data)
    ccall((:DAQmxGetReadOverWrite, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetReadOverWrite(taskHandle, data)
    ccall((:DAQmxSetReadOverWrite, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetReadOverWrite(taskHandle)
    ccall((:DAQmxResetReadOverWrite, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingFilePath(taskHandle, data, bufferSize)
    ccall((:DAQmxGetLoggingFilePath, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetLoggingFilePath(taskHandle, data)
    ccall((:DAQmxSetLoggingFilePath, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetLoggingFilePath(taskHandle)
    ccall((:DAQmxResetLoggingFilePath, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingMode(taskHandle, data)
    ccall((:DAQmxGetLoggingMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetLoggingMode(taskHandle, data)
    ccall((:DAQmxSetLoggingMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetLoggingMode(taskHandle)
    ccall((:DAQmxResetLoggingMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingTDMSGroupName(taskHandle, data, bufferSize)
    ccall((:DAQmxGetLoggingTDMSGroupName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetLoggingTDMSGroupName(taskHandle, data)
    ccall((:DAQmxSetLoggingTDMSGroupName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetLoggingTDMSGroupName(taskHandle)
    ccall((:DAQmxResetLoggingTDMSGroupName, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingTDMSOperation(taskHandle, data)
    ccall((:DAQmxGetLoggingTDMSOperation, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetLoggingTDMSOperation(taskHandle, data)
    ccall((:DAQmxSetLoggingTDMSOperation, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetLoggingTDMSOperation(taskHandle)
    ccall((:DAQmxResetLoggingTDMSOperation, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingPause(taskHandle, data)
    ccall((:DAQmxGetLoggingPause, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetLoggingPause(taskHandle, data)
    ccall((:DAQmxSetLoggingPause, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetLoggingPause(taskHandle)
    ccall((:DAQmxResetLoggingPause, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingSampsPerFile(taskHandle, data)
    ccall((:DAQmxGetLoggingSampsPerFile, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxSetLoggingSampsPerFile(taskHandle, data)
    ccall((:DAQmxSetLoggingSampsPerFile, nidaqmx), int32, (TaskHandle, uInt64), taskHandle, data)
end

function DAQmxResetLoggingSampsPerFile(taskHandle)
    ccall((:DAQmxResetLoggingSampsPerFile, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingFileWriteSize(taskHandle, data)
    ccall((:DAQmxGetLoggingFileWriteSize, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetLoggingFileWriteSize(taskHandle, data)
    ccall((:DAQmxSetLoggingFileWriteSize, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetLoggingFileWriteSize(taskHandle)
    ccall((:DAQmxResetLoggingFileWriteSize, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetLoggingFilePreallocationSize(taskHandle, data)
    ccall((:DAQmxGetLoggingFilePreallocationSize, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxSetLoggingFilePreallocationSize(taskHandle, data)
    ccall((:DAQmxSetLoggingFilePreallocationSize, nidaqmx), int32, (TaskHandle, uInt64), taskHandle, data)
end

function DAQmxResetLoggingFilePreallocationSize(taskHandle)
    ccall((:DAQmxResetLoggingFilePreallocationSize, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadCurrReadPos(taskHandle, data)
    ccall((:DAQmxGetReadCurrReadPos, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxGetReadAvailSampPerChan(taskHandle, data)
    ccall((:DAQmxGetReadAvailSampPerChan, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetReadTotalSampPerChanAcquired(taskHandle, data)
    ccall((:DAQmxGetReadTotalSampPerChanAcquired, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxGetReadCommonModeRangeErrorChansExist(taskHandle, data)
    ccall((:DAQmxGetReadCommonModeRangeErrorChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadCommonModeRangeErrorChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadCommonModeRangeErrorChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadExcitFaultChansExist(taskHandle, data)
    ccall((:DAQmxGetReadExcitFaultChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadExcitFaultChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadExcitFaultChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOvercurrentChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOvercurrentChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOvercurrentChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOvercurrentChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOvertemperatureChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOvertemperatureChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOvertemperatureChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOvertemperatureChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOpenChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOpenChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOpenChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOpenChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOpenChansDetails(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOpenChansDetails, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOpenCurrentLoopChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOpenCurrentLoopChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOpenCurrentLoopChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOpenCurrentLoopChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOpenThrmcplChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOpenThrmcplChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOpenThrmcplChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOpenThrmcplChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadOverloadedChansExist(taskHandle, data)
    ccall((:DAQmxGetReadOverloadedChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadOverloadedChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadOverloadedChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadInputLimitsFaultChansExist(taskHandle, data)
    ccall((:DAQmxGetReadInputLimitsFaultChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadInputLimitsFaultChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadInputLimitsFaultChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadPLLUnlockedChansExist(taskHandle, data)
    ccall((:DAQmxGetReadPLLUnlockedChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadPLLUnlockedChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadPLLUnlockedChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadPowerSupplyFaultChansExist(taskHandle, data)
    ccall((:DAQmxGetReadPowerSupplyFaultChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadPowerSupplyFaultChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadPowerSupplyFaultChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadSyncUnlockedChansExist(taskHandle, data)
    ccall((:DAQmxGetReadSyncUnlockedChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadSyncUnlockedChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadSyncUnlockedChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadAccessoryInsertionOrRemovalDetected(taskHandle, data)
    ccall((:DAQmxGetReadAccessoryInsertionOrRemovalDetected, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadDevsWithInsertedOrRemovedAccessories(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReadDevsWithInsertedOrRemovedAccessories, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetRemoteSenseErrorChansExist(taskHandle, data)
    ccall((:DAQmxGetRemoteSenseErrorChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetRemoteSenseErrorChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetRemoteSenseErrorChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetAuxPowerErrorChansExist(taskHandle, data)
    ccall((:DAQmxGetAuxPowerErrorChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetAuxPowerErrorChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAuxPowerErrorChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReverseVoltageErrorChansExist(taskHandle, data)
    ccall((:DAQmxGetReverseVoltageErrorChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReverseVoltageErrorChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetReverseVoltageErrorChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetReadChangeDetectHasOverflowed(taskHandle, data)
    ccall((:DAQmxGetReadChangeDetectHasOverflowed, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetReadRawDataWidth(taskHandle, data)
    ccall((:DAQmxGetReadRawDataWidth, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetReadNumChans(taskHandle, data)
    ccall((:DAQmxGetReadNumChans, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetReadDigitalLinesBytesPerChan(taskHandle, data)
    ccall((:DAQmxGetReadDigitalLinesBytesPerChan, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetReadWaitMode(taskHandle, data)
    ccall((:DAQmxGetReadWaitMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetReadWaitMode(taskHandle, data)
    ccall((:DAQmxSetReadWaitMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetReadWaitMode(taskHandle)
    ccall((:DAQmxResetReadWaitMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetReadSleepTime(taskHandle, data)
    ccall((:DAQmxGetReadSleepTime, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetReadSleepTime(taskHandle, data)
    ccall((:DAQmxSetReadSleepTime, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetReadSleepTime(taskHandle)
    ccall((:DAQmxResetReadSleepTime, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRealTimeConvLateErrorsToWarnings(taskHandle, data)
    ccall((:DAQmxGetRealTimeConvLateErrorsToWarnings, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetRealTimeConvLateErrorsToWarnings(taskHandle, data)
    ccall((:DAQmxSetRealTimeConvLateErrorsToWarnings, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetRealTimeConvLateErrorsToWarnings(taskHandle)
    ccall((:DAQmxResetRealTimeConvLateErrorsToWarnings, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRealTimeNumOfWarmupIters(taskHandle, data)
    ccall((:DAQmxGetRealTimeNumOfWarmupIters, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetRealTimeNumOfWarmupIters(taskHandle, data)
    ccall((:DAQmxSetRealTimeNumOfWarmupIters, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetRealTimeNumOfWarmupIters(taskHandle)
    ccall((:DAQmxResetRealTimeNumOfWarmupIters, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRealTimeWaitForNextSampClkWaitMode(taskHandle, data)
    ccall((:DAQmxGetRealTimeWaitForNextSampClkWaitMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetRealTimeWaitForNextSampClkWaitMode(taskHandle, data)
    ccall((:DAQmxSetRealTimeWaitForNextSampClkWaitMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetRealTimeWaitForNextSampClkWaitMode(taskHandle)
    ccall((:DAQmxResetRealTimeWaitForNextSampClkWaitMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRealTimeReportMissedSamp(taskHandle, data)
    ccall((:DAQmxGetRealTimeReportMissedSamp, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetRealTimeReportMissedSamp(taskHandle, data)
    ccall((:DAQmxSetRealTimeReportMissedSamp, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetRealTimeReportMissedSamp(taskHandle)
    ccall((:DAQmxResetRealTimeReportMissedSamp, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRealTimeWriteRecoveryMode(taskHandle, data)
    ccall((:DAQmxGetRealTimeWriteRecoveryMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetRealTimeWriteRecoveryMode(taskHandle, data)
    ccall((:DAQmxSetRealTimeWriteRecoveryMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetRealTimeWriteRecoveryMode(taskHandle)
    ccall((:DAQmxResetRealTimeWriteRecoveryMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetScaleDescr(scaleName, data, bufferSize)
    ccall((:DAQmxGetScaleDescr, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), scaleName, data, bufferSize)
end

function DAQmxSetScaleDescr(scaleName, data)
    ccall((:DAQmxSetScaleDescr, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), scaleName, data)
end

function DAQmxGetScaleScaledUnits(scaleName, data, bufferSize)
    ccall((:DAQmxGetScaleScaledUnits, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), scaleName, data, bufferSize)
end

function DAQmxSetScaleScaledUnits(scaleName, data)
    ccall((:DAQmxSetScaleScaledUnits, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}), scaleName, data)
end

function DAQmxGetScalePreScaledUnits(scaleName, data)
    ccall((:DAQmxGetScalePreScaledUnits, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), scaleName, data)
end

function DAQmxSetScalePreScaledUnits(scaleName, data)
    ccall((:DAQmxSetScalePreScaledUnits, nidaqmx), int32, (Ptr{Cchar}, int32), scaleName, data)
end

function DAQmxGetScaleType(scaleName, data)
    ccall((:DAQmxGetScaleType, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), scaleName, data)
end

function DAQmxGetScaleLinSlope(scaleName, data)
    ccall((:DAQmxGetScaleLinSlope, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleLinSlope(scaleName, data)
    ccall((:DAQmxSetScaleLinSlope, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScaleLinYIntercept(scaleName, data)
    ccall((:DAQmxGetScaleLinYIntercept, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleLinYIntercept(scaleName, data)
    ccall((:DAQmxSetScaleLinYIntercept, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScaleMapScaledMax(scaleName, data)
    ccall((:DAQmxGetScaleMapScaledMax, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleMapScaledMax(scaleName, data)
    ccall((:DAQmxSetScaleMapScaledMax, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScaleMapPreScaledMax(scaleName, data)
    ccall((:DAQmxGetScaleMapPreScaledMax, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleMapPreScaledMax(scaleName, data)
    ccall((:DAQmxSetScaleMapPreScaledMax, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScaleMapScaledMin(scaleName, data)
    ccall((:DAQmxGetScaleMapScaledMin, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleMapScaledMin(scaleName, data)
    ccall((:DAQmxSetScaleMapScaledMin, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScaleMapPreScaledMin(scaleName, data)
    ccall((:DAQmxGetScaleMapPreScaledMin, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), scaleName, data)
end

function DAQmxSetScaleMapPreScaledMin(scaleName, data)
    ccall((:DAQmxSetScaleMapPreScaledMin, nidaqmx), int32, (Ptr{Cchar}, float64), scaleName, data)
end

function DAQmxGetScalePolyForwardCoeff(scaleName, data, arraySizeInElements)
    ccall((:DAQmxGetScalePolyForwardCoeff, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxSetScalePolyForwardCoeff(scaleName, data, arraySizeInElements)
    ccall((:DAQmxSetScalePolyForwardCoeff, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxGetScalePolyReverseCoeff(scaleName, data, arraySizeInElements)
    ccall((:DAQmxGetScalePolyReverseCoeff, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxSetScalePolyReverseCoeff(scaleName, data, arraySizeInElements)
    ccall((:DAQmxSetScalePolyReverseCoeff, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxGetScaleTableScaledVals(scaleName, data, arraySizeInElements)
    ccall((:DAQmxGetScaleTableScaledVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxSetScaleTableScaledVals(scaleName, data, arraySizeInElements)
    ccall((:DAQmxSetScaleTableScaledVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxGetScaleTablePreScaledVals(scaleName, data, arraySizeInElements)
    ccall((:DAQmxGetScaleTablePreScaledVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxSetScaleTablePreScaledVals(scaleName, data, arraySizeInElements)
    ccall((:DAQmxSetScaleTablePreScaledVals, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}, uInt32), scaleName, data, arraySizeInElements)
end

function DAQmxGetSwitchChanUsage(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanUsage, nidaqmx), int32, (Ptr{Cchar}, Ptr{int32}), switchChannelName, data)
end

function DAQmxSetSwitchChanUsage(switchChannelName, data)
    ccall((:DAQmxSetSwitchChanUsage, nidaqmx), int32, (Ptr{Cchar}, int32), switchChannelName, data)
end

function DAQmxGetSwitchChanAnlgBusSharingEnable(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanAnlgBusSharingEnable, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), switchChannelName, data)
end

function DAQmxSetSwitchChanAnlgBusSharingEnable(switchChannelName, data)
    ccall((:DAQmxSetSwitchChanAnlgBusSharingEnable, nidaqmx), int32, (Ptr{Cchar}, bool32), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxACCarryCurrent(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxACCarryCurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxACSwitchCurrent(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxACSwitchCurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxACCarryPwr(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxACCarryPwr, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxACSwitchPwr(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxACSwitchPwr, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxDCCarryCurrent(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxDCCarryCurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxDCSwitchCurrent(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxDCSwitchCurrent, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxDCCarryPwr(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxDCCarryPwr, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxDCSwitchPwr(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxDCSwitchPwr, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxACVoltage(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxACVoltage, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanMaxDCVoltage(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanMaxDCVoltage, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanWireMode(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanWireMode, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), switchChannelName, data)
end

function DAQmxGetSwitchChanBandwidth(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanBandwidth, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchChanImpedance(switchChannelName, data)
    ccall((:DAQmxGetSwitchChanImpedance, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), switchChannelName, data)
end

function DAQmxGetSwitchDevSettlingTime(deviceName, data)
    ccall((:DAQmxGetSwitchDevSettlingTime, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), deviceName, data)
end

function DAQmxSetSwitchDevSettlingTime(deviceName, data)
    ccall((:DAQmxSetSwitchDevSettlingTime, nidaqmx), int32, (Ptr{Cchar}, float64), deviceName, data)
end

function DAQmxGetSwitchDevAutoConnAnlgBus(deviceName, data)
    ccall((:DAQmxGetSwitchDevAutoConnAnlgBus, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), deviceName, data)
end

function DAQmxSetSwitchDevAutoConnAnlgBus(deviceName, data)
    ccall((:DAQmxSetSwitchDevAutoConnAnlgBus, nidaqmx), int32, (Ptr{Cchar}, bool32), deviceName, data)
end

function DAQmxGetSwitchDevPwrDownLatchRelaysAfterSettling(deviceName, data)
    ccall((:DAQmxGetSwitchDevPwrDownLatchRelaysAfterSettling, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), deviceName, data)
end

function DAQmxSetSwitchDevPwrDownLatchRelaysAfterSettling(deviceName, data)
    ccall((:DAQmxSetSwitchDevPwrDownLatchRelaysAfterSettling, nidaqmx), int32, (Ptr{Cchar}, bool32), deviceName, data)
end

function DAQmxGetSwitchDevSettled(deviceName, data)
    ccall((:DAQmxGetSwitchDevSettled, nidaqmx), int32, (Ptr{Cchar}, Ptr{bool32}), deviceName, data)
end

function DAQmxGetSwitchDevRelayList(deviceName, data, bufferSize)
    ccall((:DAQmxGetSwitchDevRelayList, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), deviceName, data, bufferSize)
end

function DAQmxGetSwitchDevNumRelays(deviceName, data)
    ccall((:DAQmxGetSwitchDevNumRelays, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetSwitchDevSwitchChanList(deviceName, data, bufferSize)
    ccall((:DAQmxGetSwitchDevSwitchChanList, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), deviceName, data, bufferSize)
end

function DAQmxGetSwitchDevNumSwitchChans(deviceName, data)
    ccall((:DAQmxGetSwitchDevNumSwitchChans, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetSwitchDevNumRows(deviceName, data)
    ccall((:DAQmxGetSwitchDevNumRows, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetSwitchDevNumColumns(deviceName, data)
    ccall((:DAQmxGetSwitchDevNumColumns, nidaqmx), int32, (Ptr{Cchar}, Ptr{uInt32}), deviceName, data)
end

function DAQmxGetSwitchDevTopology(deviceName, data, bufferSize)
    ccall((:DAQmxGetSwitchDevTopology, nidaqmx), int32, (Ptr{Cchar}, Ptr{Cchar}, uInt32), deviceName, data, bufferSize)
end

function DAQmxGetSwitchDevTemperature(deviceName, data)
    ccall((:DAQmxGetSwitchDevTemperature, nidaqmx), int32, (Ptr{Cchar}, Ptr{float64}), deviceName, data)
end

function DAQmxGetSwitchScanBreakMode(taskHandle, data)
    ccall((:DAQmxGetSwitchScanBreakMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSwitchScanBreakMode(taskHandle, data)
    ccall((:DAQmxSetSwitchScanBreakMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSwitchScanBreakMode(taskHandle)
    ccall((:DAQmxResetSwitchScanBreakMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSwitchScanRepeatMode(taskHandle, data)
    ccall((:DAQmxGetSwitchScanRepeatMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSwitchScanRepeatMode(taskHandle, data)
    ccall((:DAQmxSetSwitchScanRepeatMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSwitchScanRepeatMode(taskHandle)
    ccall((:DAQmxResetSwitchScanRepeatMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSwitchScanWaitingForAdv(taskHandle, data)
    ccall((:DAQmxGetSwitchScanWaitingForAdv, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetSysGlobalChans(data, bufferSize)
    ccall((:DAQmxGetSysGlobalChans, nidaqmx), int32, (Ptr{Cchar}, uInt32), data, bufferSize)
end

function DAQmxGetSysScales(data, bufferSize)
    ccall((:DAQmxGetSysScales, nidaqmx), int32, (Ptr{Cchar}, uInt32), data, bufferSize)
end

function DAQmxGetSysTasks(data, bufferSize)
    ccall((:DAQmxGetSysTasks, nidaqmx), int32, (Ptr{Cchar}, uInt32), data, bufferSize)
end

function DAQmxGetSysDevNames(data, bufferSize)
    ccall((:DAQmxGetSysDevNames, nidaqmx), int32, (Ptr{Cchar}, uInt32), data, bufferSize)
end

function DAQmxGetSysNIDAQMajorVersion(data)
    ccall((:DAQmxGetSysNIDAQMajorVersion, nidaqmx), int32, (Ptr{uInt32},), data)
end

function DAQmxGetSysNIDAQMinorVersion(data)
    ccall((:DAQmxGetSysNIDAQMinorVersion, nidaqmx), int32, (Ptr{uInt32},), data)
end

function DAQmxGetSysNIDAQUpdateVersion(data)
    ccall((:DAQmxGetSysNIDAQUpdateVersion, nidaqmx), int32, (Ptr{uInt32},), data)
end

function DAQmxGetTaskName(taskHandle, data, bufferSize)
    ccall((:DAQmxGetTaskName, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetTaskChannels(taskHandle, data, bufferSize)
    ccall((:DAQmxGetTaskChannels, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetTaskNumChans(taskHandle, data)
    ccall((:DAQmxGetTaskNumChans, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetTaskDevices(taskHandle, data, bufferSize)
    ccall((:DAQmxGetTaskDevices, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetTaskNumDevices(taskHandle, data)
    ccall((:DAQmxGetTaskNumDevices, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetTaskComplete(taskHandle, data)
    ccall((:DAQmxGetTaskComplete, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetSampQuantSampMode(taskHandle, data)
    ccall((:DAQmxGetSampQuantSampMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampQuantSampMode(taskHandle, data)
    ccall((:DAQmxSetSampQuantSampMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampQuantSampMode(taskHandle)
    ccall((:DAQmxResetSampQuantSampMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampQuantSampPerChan(taskHandle, data)
    ccall((:DAQmxGetSampQuantSampPerChan, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxSetSampQuantSampPerChan(taskHandle, data)
    ccall((:DAQmxSetSampQuantSampPerChan, nidaqmx), int32, (TaskHandle, uInt64), taskHandle, data)
end

function DAQmxResetSampQuantSampPerChan(taskHandle)
    ccall((:DAQmxResetSampQuantSampPerChan, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampTimingType(taskHandle, data)
    ccall((:DAQmxGetSampTimingType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampTimingType(taskHandle, data)
    ccall((:DAQmxSetSampTimingType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampTimingType(taskHandle)
    ccall((:DAQmxResetSampTimingType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkRate(taskHandle, data)
    ccall((:DAQmxGetSampClkRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSampClkRate(taskHandle, data)
    ccall((:DAQmxSetSampClkRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSampClkRate(taskHandle)
    ccall((:DAQmxResetSampClkRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkMaxRate(taskHandle, data)
    ccall((:DAQmxGetSampClkMaxRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxGetSampClkSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSampClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetSampClkSrc(taskHandle, data)
    ccall((:DAQmxSetSampClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetSampClkSrc(taskHandle)
    ccall((:DAQmxResetSampClkSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkActiveEdge(taskHandle, data)
    ccall((:DAQmxGetSampClkActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampClkActiveEdge(taskHandle, data)
    ccall((:DAQmxSetSampClkActiveEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampClkActiveEdge(taskHandle)
    ccall((:DAQmxResetSampClkActiveEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkOverrunBehavior(taskHandle, data)
    ccall((:DAQmxGetSampClkOverrunBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampClkOverrunBehavior(taskHandle, data)
    ccall((:DAQmxSetSampClkOverrunBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampClkOverrunBehavior(taskHandle)
    ccall((:DAQmxResetSampClkOverrunBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkUnderflowBehavior(taskHandle, data)
    ccall((:DAQmxGetSampClkUnderflowBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampClkUnderflowBehavior(taskHandle, data)
    ccall((:DAQmxSetSampClkUnderflowBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampClkUnderflowBehavior(taskHandle)
    ccall((:DAQmxResetSampClkUnderflowBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTimebaseDiv(taskHandle, data)
    ccall((:DAQmxGetSampClkTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetSampClkTimebaseDiv(taskHandle, data)
    ccall((:DAQmxSetSampClkTimebaseDiv, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetSampClkTimebaseDiv(taskHandle)
    ccall((:DAQmxResetSampClkTimebaseDiv, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSampClkTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetSampClkTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetSampClkTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSampClkTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetSampClkTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSampClkTimebaseRate(taskHandle)
    ccall((:DAQmxResetSampClkTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSampClkTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetSampClkTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetSampClkTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetSampClkTimebaseSrc(taskHandle)
    ccall((:DAQmxResetSampClkTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTimebaseActiveEdge(taskHandle, data)
    ccall((:DAQmxGetSampClkTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampClkTimebaseActiveEdge(taskHandle, data)
    ccall((:DAQmxSetSampClkTimebaseActiveEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampClkTimebaseActiveEdge(taskHandle)
    ccall((:DAQmxResetSampClkTimebaseActiveEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTimebaseMasterTimebaseDiv(taskHandle, data)
    ccall((:DAQmxGetSampClkTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetSampClkTimebaseMasterTimebaseDiv(taskHandle, data)
    ccall((:DAQmxSetSampClkTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetSampClkTimebaseMasterTimebaseDiv(taskHandle)
    ccall((:DAQmxResetSampClkTimebaseMasterTimebaseDiv, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkTimebaseTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSampClkTimebaseTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetSampClkDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetSampClkDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetSampClkDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetSampClkDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetSampClkDigFltrEnable(taskHandle)
    ccall((:DAQmxResetSampClkDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetSampClkDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSampClkDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetSampClkDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSampClkDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetSampClkDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSampClkDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetSampClkDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetSampClkDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetSampClkDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetSampClkDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetSampClkDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSampClkDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetSampClkDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSampClkDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetSampClkDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetSampClkDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetSampClkDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetSampClkDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetSampClkDigSyncEnable(taskHandle)
    ccall((:DAQmxResetSampClkDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampClkWriteWfmUseInitialWfmDT(taskHandle, data)
    ccall((:DAQmxGetSampClkWriteWfmUseInitialWfmDT, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetSampClkWriteWfmUseInitialWfmDT(taskHandle, data)
    ccall((:DAQmxSetSampClkWriteWfmUseInitialWfmDT, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetSampClkWriteWfmUseInitialWfmDT(taskHandle)
    ccall((:DAQmxResetSampClkWriteWfmUseInitialWfmDT, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetHshkDelayAfterXfer(taskHandle, data)
    ccall((:DAQmxGetHshkDelayAfterXfer, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetHshkDelayAfterXfer(taskHandle, data)
    ccall((:DAQmxSetHshkDelayAfterXfer, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetHshkDelayAfterXfer(taskHandle)
    ccall((:DAQmxResetHshkDelayAfterXfer, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetHshkStartCond(taskHandle, data)
    ccall((:DAQmxGetHshkStartCond, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetHshkStartCond(taskHandle, data)
    ccall((:DAQmxSetHshkStartCond, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetHshkStartCond(taskHandle)
    ccall((:DAQmxResetHshkStartCond, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetHshkSampleInputDataWhen(taskHandle, data)
    ccall((:DAQmxGetHshkSampleInputDataWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetHshkSampleInputDataWhen(taskHandle, data)
    ccall((:DAQmxSetHshkSampleInputDataWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetHshkSampleInputDataWhen(taskHandle)
    ccall((:DAQmxResetHshkSampleInputDataWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetChangeDetectDIRisingEdgePhysicalChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetChangeDetectDIRisingEdgePhysicalChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetChangeDetectDIRisingEdgePhysicalChans(taskHandle, data)
    ccall((:DAQmxSetChangeDetectDIRisingEdgePhysicalChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetChangeDetectDIRisingEdgePhysicalChans(taskHandle)
    ccall((:DAQmxResetChangeDetectDIRisingEdgePhysicalChans, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetChangeDetectDIFallingEdgePhysicalChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetChangeDetectDIFallingEdgePhysicalChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetChangeDetectDIFallingEdgePhysicalChans(taskHandle, data)
    ccall((:DAQmxSetChangeDetectDIFallingEdgePhysicalChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetChangeDetectDIFallingEdgePhysicalChans(taskHandle)
    ccall((:DAQmxResetChangeDetectDIFallingEdgePhysicalChans, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetChangeDetectDITristate(taskHandle, data)
    ccall((:DAQmxGetChangeDetectDITristate, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetChangeDetectDITristate(taskHandle, data)
    ccall((:DAQmxSetChangeDetectDITristate, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetChangeDetectDITristate(taskHandle)
    ccall((:DAQmxResetChangeDetectDITristate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetOnDemandSimultaneousAOEnable(taskHandle, data)
    ccall((:DAQmxGetOnDemandSimultaneousAOEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetOnDemandSimultaneousAOEnable(taskHandle, data)
    ccall((:DAQmxSetOnDemandSimultaneousAOEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetOnDemandSimultaneousAOEnable(taskHandle)
    ccall((:DAQmxResetOnDemandSimultaneousAOEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetImplicitUnderflowBehavior(taskHandle, data)
    ccall((:DAQmxGetImplicitUnderflowBehavior, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetImplicitUnderflowBehavior(taskHandle, data)
    ccall((:DAQmxSetImplicitUnderflowBehavior, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetImplicitUnderflowBehavior(taskHandle)
    ccall((:DAQmxResetImplicitUnderflowBehavior, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvRate(taskHandle, data)
    ccall((:DAQmxGetAIConvRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAIConvRate(taskHandle, data)
    ccall((:DAQmxSetAIConvRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAIConvRate(taskHandle)
    ccall((:DAQmxResetAIConvRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvRateEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvRateEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvRateEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvMaxRate(taskHandle, data)
    ccall((:DAQmxGetAIConvMaxRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxGetAIConvMaxRateEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvMaxRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, deviceNames, data)
end

function DAQmxGetAIConvSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAIConvSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAIConvSrc(taskHandle, data)
    ccall((:DAQmxSetAIConvSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAIConvSrc(taskHandle)
    ccall((:DAQmxResetAIConvSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvSrcEx(taskHandle, deviceNames, data, bufferSize)
    ccall((:DAQmxGetAIConvSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, deviceNames, data, bufferSize)
end

function DAQmxSetAIConvSrcEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvSrcEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvActiveEdge(taskHandle, data)
    ccall((:DAQmxGetAIConvActiveEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAIConvActiveEdge(taskHandle, data)
    ccall((:DAQmxSetAIConvActiveEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAIConvActiveEdge(taskHandle)
    ccall((:DAQmxResetAIConvActiveEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvActiveEdgeEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvActiveEdgeEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvActiveEdgeEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvActiveEdgeEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvActiveEdgeEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvActiveEdgeEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvTimebaseDiv(taskHandle, data)
    ccall((:DAQmxGetAIConvTimebaseDiv, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetAIConvTimebaseDiv(taskHandle, data)
    ccall((:DAQmxSetAIConvTimebaseDiv, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetAIConvTimebaseDiv(taskHandle)
    ccall((:DAQmxResetAIConvTimebaseDiv, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvTimebaseDivEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvTimebaseDivEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{uInt32}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvTimebaseDivEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvTimebaseDivEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvTimebaseDivEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvTimebaseDivEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvTimebaseSrc(taskHandle, data)
    ccall((:DAQmxGetAIConvTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAIConvTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAIConvTimebaseSrc, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAIConvTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAIConvTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvTimebaseSrcEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvTimebaseSrcEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvTimebaseSrcEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetDelayFromSampClkDelayUnits(taskHandle, data)
    ccall((:DAQmxGetDelayFromSampClkDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDelayFromSampClkDelayUnits(taskHandle, data)
    ccall((:DAQmxSetDelayFromSampClkDelayUnits, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDelayFromSampClkDelayUnits(taskHandle)
    ccall((:DAQmxResetDelayFromSampClkDelayUnits, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDelayFromSampClkDelayUnitsEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetDelayFromSampClkDelayUnitsEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, deviceNames, data)
end

function DAQmxSetDelayFromSampClkDelayUnitsEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetDelayFromSampClkDelayUnitsEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, deviceNames, data)
end

function DAQmxResetDelayFromSampClkDelayUnitsEx(taskHandle, deviceNames)
    ccall((:DAQmxResetDelayFromSampClkDelayUnitsEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetDelayFromSampClkDelay(taskHandle, data)
    ccall((:DAQmxGetDelayFromSampClkDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDelayFromSampClkDelay(taskHandle, data)
    ccall((:DAQmxSetDelayFromSampClkDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDelayFromSampClkDelay(taskHandle)
    ccall((:DAQmxResetDelayFromSampClkDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDelayFromSampClkDelayEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetDelayFromSampClkDelayEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, deviceNames, data)
end

function DAQmxSetDelayFromSampClkDelayEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetDelayFromSampClkDelayEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, deviceNames, data)
end

function DAQmxResetDelayFromSampClkDelayEx(taskHandle, deviceNames)
    ccall((:DAQmxResetDelayFromSampClkDelayEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAIConvDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAIConvDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAIConvDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAIConvDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAIConvDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvDigFltrEnableEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvDigFltrEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvDigFltrEnableEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvDigFltrEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvDigFltrEnableEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvDigFltrEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAIConvDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAIConvDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAIConvDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAIConvDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAIConvDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvDigFltrMinPulseWidthEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvDigFltrMinPulseWidthEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvDigFltrMinPulseWidthEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvDigFltrMinPulseWidthEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvDigFltrMinPulseWidthEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvDigFltrMinPulseWidthEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAIConvDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAIConvDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAIConvDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAIConvDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAIConvDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvDigFltrTimebaseSrcEx(taskHandle, deviceNames, data, bufferSize)
    ccall((:DAQmxGetAIConvDigFltrTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}, uInt32), taskHandle, deviceNames, data, bufferSize)
end

function DAQmxSetAIConvDigFltrTimebaseSrcEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvDigFltrTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{Cchar}), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvDigFltrTimebaseSrcEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvDigFltrTimebaseSrcEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAIConvDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAIConvDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAIConvDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAIConvDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAIConvDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvDigFltrTimebaseRateEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvDigFltrTimebaseRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvDigFltrTimebaseRateEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvDigFltrTimebaseRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvDigFltrTimebaseRateEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvDigFltrTimebaseRateEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetAIConvDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAIConvDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAIConvDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAIConvDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAIConvDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAIConvDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAIConvDigSyncEnableEx(taskHandle, deviceNames, data)
    ccall((:DAQmxGetAIConvDigSyncEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{bool32}), taskHandle, deviceNames, data)
end

function DAQmxSetAIConvDigSyncEnableEx(taskHandle, deviceNames, data)
    ccall((:DAQmxSetAIConvDigSyncEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, bool32), taskHandle, deviceNames, data)
end

function DAQmxResetAIConvDigSyncEnableEx(taskHandle, deviceNames)
    ccall((:DAQmxResetAIConvDigSyncEnableEx, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, deviceNames)
end

function DAQmxGetMasterTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetMasterTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetMasterTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetMasterTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetMasterTimebaseRate(taskHandle)
    ccall((:DAQmxResetMasterTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetMasterTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetMasterTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetMasterTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetMasterTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetMasterTimebaseSrc(taskHandle)
    ccall((:DAQmxResetMasterTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefClkRate(taskHandle, data)
    ccall((:DAQmxGetRefClkRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetRefClkRate(taskHandle, data)
    ccall((:DAQmxSetRefClkRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetRefClkRate(taskHandle)
    ccall((:DAQmxResetRefClkRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefClkSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetRefClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetRefClkSrc(taskHandle, data)
    ccall((:DAQmxSetRefClkSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetRefClkSrc(taskHandle)
    ccall((:DAQmxResetRefClkSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseType(taskHandle, data)
    ccall((:DAQmxGetSyncPulseType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSyncPulseType(taskHandle, data)
    ccall((:DAQmxSetSyncPulseType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSyncPulseType(taskHandle)
    ccall((:DAQmxResetSyncPulseType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSyncPulseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetSyncPulseSrc(taskHandle, data)
    ccall((:DAQmxSetSyncPulseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetSyncPulseSrc(taskHandle)
    ccall((:DAQmxResetSyncPulseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseTimeWhen(taskHandle, data)
    ccall((:DAQmxGetSyncPulseTimeWhen, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxSetSyncPulseTimeWhen(taskHandle, data)
    ccall((:DAQmxSetSyncPulseTimeWhen, nidaqmx), int32, (TaskHandle, CVIAbsoluteTime), taskHandle, data)
end

function DAQmxResetSyncPulseTimeWhen(taskHandle)
    ccall((:DAQmxResetSyncPulseTimeWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseTimeTimescale(taskHandle, data)
    ccall((:DAQmxGetSyncPulseTimeTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSyncPulseTimeTimescale(taskHandle, data)
    ccall((:DAQmxSetSyncPulseTimeTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSyncPulseTimeTimescale(taskHandle)
    ccall((:DAQmxResetSyncPulseTimeTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseSyncTime(taskHandle, data)
    ccall((:DAQmxGetSyncPulseSyncTime, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxGetSyncPulseMinDelayToStart(taskHandle, data)
    ccall((:DAQmxGetSyncPulseMinDelayToStart, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSyncPulseMinDelayToStart(taskHandle, data)
    ccall((:DAQmxSetSyncPulseMinDelayToStart, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSyncPulseMinDelayToStart(taskHandle)
    ccall((:DAQmxResetSyncPulseMinDelayToStart, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseResetTime(taskHandle, data)
    ccall((:DAQmxGetSyncPulseResetTime, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxGetSyncPulseResetDelay(taskHandle, data)
    ccall((:DAQmxGetSyncPulseResetDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetSyncPulseResetDelay(taskHandle, data)
    ccall((:DAQmxSetSyncPulseResetDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetSyncPulseResetDelay(taskHandle)
    ccall((:DAQmxResetSyncPulseResetDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSyncPulseTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetSyncPulseTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetSyncClkInterval(taskHandle, data)
    ccall((:DAQmxGetSyncClkInterval, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetSyncClkInterval(taskHandle, data)
    ccall((:DAQmxSetSyncClkInterval, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetSyncClkInterval(taskHandle)
    ccall((:DAQmxResetSyncClkInterval, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetSampTimingEngine(taskHandle, data)
    ccall((:DAQmxGetSampTimingEngine, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetSampTimingEngine(taskHandle, data)
    ccall((:DAQmxSetSampTimingEngine, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetSampTimingEngine(taskHandle)
    ccall((:DAQmxResetSampTimingEngine, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetFirstSampTimestampEnable(taskHandle, data)
    ccall((:DAQmxGetFirstSampTimestampEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetFirstSampTimestampEnable(taskHandle, data)
    ccall((:DAQmxSetFirstSampTimestampEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetFirstSampTimestampEnable(taskHandle)
    ccall((:DAQmxResetFirstSampTimestampEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetFirstSampTimestampTimescale(taskHandle, data)
    ccall((:DAQmxGetFirstSampTimestampTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetFirstSampTimestampTimescale(taskHandle, data)
    ccall((:DAQmxSetFirstSampTimestampTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetFirstSampTimestampTimescale(taskHandle)
    ccall((:DAQmxResetFirstSampTimestampTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetFirstSampTimestampVal(taskHandle, data)
    ccall((:DAQmxGetFirstSampTimestampVal, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxGetFirstSampClkWhen(taskHandle, data)
    ccall((:DAQmxGetFirstSampClkWhen, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxSetFirstSampClkWhen(taskHandle, data)
    ccall((:DAQmxSetFirstSampClkWhen, nidaqmx), int32, (TaskHandle, CVIAbsoluteTime), taskHandle, data)
end

function DAQmxResetFirstSampClkWhen(taskHandle)
    ccall((:DAQmxResetFirstSampClkWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetFirstSampClkTimescale(taskHandle, data)
    ccall((:DAQmxGetFirstSampClkTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetFirstSampClkTimescale(taskHandle, data)
    ccall((:DAQmxSetFirstSampClkTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetFirstSampClkTimescale(taskHandle)
    ccall((:DAQmxResetFirstSampClkTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetFirstSampClkOffset(taskHandle, data)
    ccall((:DAQmxGetFirstSampClkOffset, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetFirstSampClkOffset(taskHandle, data)
    ccall((:DAQmxSetFirstSampClkOffset, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetFirstSampClkOffset(taskHandle)
    ccall((:DAQmxResetFirstSampClkOffset, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigType(taskHandle, data)
    ccall((:DAQmxGetStartTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetStartTrigType(taskHandle, data)
    ccall((:DAQmxSetStartTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetStartTrigType(taskHandle)
    ccall((:DAQmxResetStartTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetStartTrigTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetDigEdgeStartTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeStartTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigEdge(taskHandle, data)
    ccall((:DAQmxGetDigEdgeStartTrigEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigEdgeStartTrigEdge(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigEdge(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetDigEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeStartTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetDigEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeStartTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternStartTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternStartTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigPatternStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternStartTrigSrc(taskHandle)
    ccall((:DAQmxResetDigPatternStartTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternStartTrigPattern(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternStartTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternStartTrigPattern(taskHandle, data)
    ccall((:DAQmxSetDigPatternStartTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternStartTrigPattern(taskHandle)
    ccall((:DAQmxResetDigPatternStartTrigPattern, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternStartTrigWhen(taskHandle, data)
    ccall((:DAQmxGetDigPatternStartTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigPatternStartTrigWhen(taskHandle, data)
    ccall((:DAQmxSetDigPatternStartTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigPatternStartTrigWhen(taskHandle)
    ccall((:DAQmxResetDigPatternStartTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgEdgeStartTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigSlope(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigSlope, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigSlope(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigSlope, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigSlope(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigSlope, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigLvl(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigLvl, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigLvl(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigLvl, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigLvl(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigHyst(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigHyst, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigHyst(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigHyst, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigHyst(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigHyst, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgEdgeStartTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgEdgeStartTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgEdgeStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeStartTrigSrcs(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgMultiEdgeStartTrigSrcs, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgMultiEdgeStartTrigSrcs(taskHandle, data)
    ccall((:DAQmxSetAnlgMultiEdgeStartTrigSrcs, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgMultiEdgeStartTrigSrcs(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeStartTrigSrcs, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeStartTrigSlopes(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeStartTrigSlopes, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeStartTrigSlopes(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeStartTrigSlopes, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeStartTrigSlopes(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeStartTrigSlopes, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeStartTrigLvls(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeStartTrigLvls, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeStartTrigLvls(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeStartTrigLvls, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeStartTrigLvls(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeStartTrigLvls, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeStartTrigHysts(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeStartTrigHysts, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeStartTrigHysts(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeStartTrigHysts, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeStartTrigHysts(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeStartTrigHysts, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeStartTrigCouplings(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeStartTrigCouplings, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeStartTrigCouplings(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeStartTrigCouplings, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeStartTrigCouplings(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeStartTrigCouplings, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinStartTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigWhen(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigWhen(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigWhen(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigTop(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigTop, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigTop(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigTop, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigTop(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigTop, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigBtm(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigBtm, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigBtm(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigBtm, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigBtm(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigBtm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinStartTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinStartTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTrigWhen(taskHandle, data)
    ccall((:DAQmxGetStartTrigTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxSetStartTrigTrigWhen(taskHandle, data)
    ccall((:DAQmxSetStartTrigTrigWhen, nidaqmx), int32, (TaskHandle, CVIAbsoluteTime), taskHandle, data)
end

function DAQmxResetStartTrigTrigWhen(taskHandle)
    ccall((:DAQmxResetStartTrigTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTimescale(taskHandle, data)
    ccall((:DAQmxGetStartTrigTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetStartTrigTimescale(taskHandle, data)
    ccall((:DAQmxSetStartTrigTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetStartTrigTimescale(taskHandle)
    ccall((:DAQmxResetStartTrigTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxGetStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetStartTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxSetStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetStartTrigTimestampEnable(taskHandle)
    ccall((:DAQmxResetStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxGetStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetStartTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxSetStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetStartTrigTimestampTimescale(taskHandle)
    ccall((:DAQmxResetStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTimestampVal(taskHandle, data)
    ccall((:DAQmxGetStartTrigTimestampVal, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxGetStartTrigDelay(taskHandle, data)
    ccall((:DAQmxGetStartTrigDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetStartTrigDelay(taskHandle, data)
    ccall((:DAQmxSetStartTrigDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetStartTrigDelay(taskHandle)
    ccall((:DAQmxResetStartTrigDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigDelayUnits(taskHandle, data)
    ccall((:DAQmxGetStartTrigDelayUnits, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetStartTrigDelayUnits(taskHandle, data)
    ccall((:DAQmxSetStartTrigDelayUnits, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetStartTrigDelayUnits(taskHandle)
    ccall((:DAQmxResetStartTrigDelayUnits, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigRetriggerable(taskHandle, data)
    ccall((:DAQmxGetStartTrigRetriggerable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetStartTrigRetriggerable(taskHandle, data)
    ccall((:DAQmxSetStartTrigRetriggerable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetStartTrigRetriggerable(taskHandle)
    ccall((:DAQmxResetStartTrigRetriggerable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigTrigWin(taskHandle, data)
    ccall((:DAQmxGetStartTrigTrigWin, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetStartTrigTrigWin(taskHandle, data)
    ccall((:DAQmxSetStartTrigTrigWin, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetStartTrigTrigWin(taskHandle)
    ccall((:DAQmxResetStartTrigTrigWin, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigRetriggerWin(taskHandle, data)
    ccall((:DAQmxGetStartTrigRetriggerWin, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetStartTrigRetriggerWin(taskHandle, data)
    ccall((:DAQmxSetStartTrigRetriggerWin, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetStartTrigRetriggerWin(taskHandle)
    ccall((:DAQmxResetStartTrigRetriggerWin, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetStartTrigMaxNumTrigsToDetect(taskHandle, data)
    ccall((:DAQmxGetStartTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetStartTrigMaxNumTrigsToDetect(taskHandle, data)
    ccall((:DAQmxSetStartTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetStartTrigMaxNumTrigsToDetect(taskHandle)
    ccall((:DAQmxResetStartTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigType(taskHandle, data)
    ccall((:DAQmxGetRefTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetRefTrigType(taskHandle, data)
    ccall((:DAQmxSetRefTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetRefTrigType(taskHandle)
    ccall((:DAQmxResetRefTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigPretrigSamples(taskHandle, data)
    ccall((:DAQmxGetRefTrigPretrigSamples, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetRefTrigPretrigSamples(taskHandle, data)
    ccall((:DAQmxSetRefTrigPretrigSamples, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetRefTrigPretrigSamples(taskHandle)
    ccall((:DAQmxResetRefTrigPretrigSamples, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetRefTrigTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetDigEdgeRefTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeRefTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigEdge(taskHandle, data)
    ccall((:DAQmxGetDigEdgeRefTrigEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigEdgeRefTrigEdge(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigEdge(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetDigEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeRefTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetDigEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeRefTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternRefTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternRefTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigPatternRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternRefTrigSrc(taskHandle)
    ccall((:DAQmxResetDigPatternRefTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternRefTrigPattern(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternRefTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternRefTrigPattern(taskHandle, data)
    ccall((:DAQmxSetDigPatternRefTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternRefTrigPattern(taskHandle)
    ccall((:DAQmxResetDigPatternRefTrigPattern, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternRefTrigWhen(taskHandle, data)
    ccall((:DAQmxGetDigPatternRefTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigPatternRefTrigWhen(taskHandle, data)
    ccall((:DAQmxSetDigPatternRefTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigPatternRefTrigWhen(taskHandle)
    ccall((:DAQmxResetDigPatternRefTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgEdgeRefTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigSlope(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigSlope, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigSlope(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigSlope, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigSlope(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigSlope, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigLvl(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigLvl, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigLvl(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigLvl, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigLvl(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigHyst(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigHyst, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigHyst(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigHyst, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigHyst(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigHyst, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgEdgeRefTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgEdgeRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgEdgeRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgEdgeRefTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgEdgeRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeRefTrigSrcs(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgMultiEdgeRefTrigSrcs, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgMultiEdgeRefTrigSrcs(taskHandle, data)
    ccall((:DAQmxSetAnlgMultiEdgeRefTrigSrcs, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgMultiEdgeRefTrigSrcs(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeRefTrigSrcs, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeRefTrigSlopes(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeRefTrigSlopes, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeRefTrigSlopes(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeRefTrigSlopes, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeRefTrigSlopes(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeRefTrigSlopes, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeRefTrigLvls(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeRefTrigLvls, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeRefTrigLvls(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeRefTrigLvls, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeRefTrigLvls(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeRefTrigLvls, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeRefTrigHysts(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeRefTrigHysts, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeRefTrigHysts(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeRefTrigHysts, nidaqmx), int32, (TaskHandle, Ptr{float64}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeRefTrigHysts(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeRefTrigHysts, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgMultiEdgeRefTrigCouplings(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxGetAnlgMultiEdgeRefTrigCouplings, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxSetAnlgMultiEdgeRefTrigCouplings(taskHandle, data, arraySizeInElements)
    ccall((:DAQmxSetAnlgMultiEdgeRefTrigCouplings, nidaqmx), int32, (TaskHandle, Ptr{int32}, uInt32), taskHandle, data, arraySizeInElements)
end

function DAQmxResetAnlgMultiEdgeRefTrigCouplings(taskHandle)
    ccall((:DAQmxResetAnlgMultiEdgeRefTrigCouplings, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinRefTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigWhen(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigWhen(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigWhen(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigTop(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigTop, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigTop(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigTop, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigTop(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigTop, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigBtm(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigBtm, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigBtm(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigBtm, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigBtm(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigBtm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinRefTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinRefTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinRefTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinRefTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigAutoTrigEnable(taskHandle, data)
    ccall((:DAQmxGetRefTrigAutoTrigEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetRefTrigAutoTrigEnable(taskHandle, data)
    ccall((:DAQmxSetRefTrigAutoTrigEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetRefTrigAutoTrigEnable(taskHandle)
    ccall((:DAQmxResetRefTrigAutoTrigEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigAutoTriggered(taskHandle, data)
    ccall((:DAQmxGetRefTrigAutoTriggered, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetRefTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxGetRefTrigTimestampEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetRefTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxSetRefTrigTimestampEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetRefTrigTimestampEnable(taskHandle)
    ccall((:DAQmxResetRefTrigTimestampEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxGetRefTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetRefTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxSetRefTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetRefTrigTimestampTimescale(taskHandle)
    ccall((:DAQmxResetRefTrigTimestampTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigTimestampVal(taskHandle, data)
    ccall((:DAQmxGetRefTrigTimestampVal, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxGetRefTrigDelay(taskHandle, data)
    ccall((:DAQmxGetRefTrigDelay, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetRefTrigDelay(taskHandle, data)
    ccall((:DAQmxSetRefTrigDelay, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetRefTrigDelay(taskHandle)
    ccall((:DAQmxResetRefTrigDelay, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigRetriggerable(taskHandle, data)
    ccall((:DAQmxGetRefTrigRetriggerable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetRefTrigRetriggerable(taskHandle, data)
    ccall((:DAQmxSetRefTrigRetriggerable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetRefTrigRetriggerable(taskHandle)
    ccall((:DAQmxResetRefTrigRetriggerable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigTrigWin(taskHandle, data)
    ccall((:DAQmxGetRefTrigTrigWin, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetRefTrigTrigWin(taskHandle, data)
    ccall((:DAQmxSetRefTrigTrigWin, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetRefTrigTrigWin(taskHandle)
    ccall((:DAQmxResetRefTrigTrigWin, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigRetriggerWin(taskHandle, data)
    ccall((:DAQmxGetRefTrigRetriggerWin, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetRefTrigRetriggerWin(taskHandle, data)
    ccall((:DAQmxSetRefTrigRetriggerWin, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetRefTrigRetriggerWin(taskHandle)
    ccall((:DAQmxResetRefTrigRetriggerWin, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetRefTrigMaxNumTrigsToDetect(taskHandle, data)
    ccall((:DAQmxGetRefTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxSetRefTrigMaxNumTrigsToDetect(taskHandle, data)
    ccall((:DAQmxSetRefTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle, uInt32), taskHandle, data)
end

function DAQmxResetRefTrigMaxNumTrigsToDetect(taskHandle)
    ccall((:DAQmxResetRefTrigMaxNumTrigsToDetect, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAdvTrigType(taskHandle, data)
    ccall((:DAQmxGetAdvTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAdvTrigType(taskHandle, data)
    ccall((:DAQmxSetAdvTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAdvTrigType(taskHandle)
    ccall((:DAQmxResetAdvTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeAdvTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeAdvTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeAdvTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeAdvTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeAdvTrigSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeAdvTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeAdvTrigEdge(taskHandle, data)
    ccall((:DAQmxGetDigEdgeAdvTrigEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigEdgeAdvTrigEdge(taskHandle, data)
    ccall((:DAQmxSetDigEdgeAdvTrigEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigEdgeAdvTrigEdge(taskHandle)
    ccall((:DAQmxResetDigEdgeAdvTrigEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeAdvTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeAdvTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeAdvTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeAdvTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeAdvTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeAdvTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetHshkTrigType(taskHandle, data)
    ccall((:DAQmxGetHshkTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetHshkTrigType(taskHandle, data)
    ccall((:DAQmxSetHshkTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetHshkTrigType(taskHandle)
    ccall((:DAQmxResetHshkTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetInterlockedHshkTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetInterlockedHshkTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetInterlockedHshkTrigSrc(taskHandle, data)
    ccall((:DAQmxSetInterlockedHshkTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetInterlockedHshkTrigSrc(taskHandle)
    ccall((:DAQmxResetInterlockedHshkTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetInterlockedHshkTrigAssertedLvl(taskHandle, data)
    ccall((:DAQmxGetInterlockedHshkTrigAssertedLvl, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetInterlockedHshkTrigAssertedLvl(taskHandle, data)
    ccall((:DAQmxSetInterlockedHshkTrigAssertedLvl, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetInterlockedHshkTrigAssertedLvl(taskHandle)
    ccall((:DAQmxResetInterlockedHshkTrigAssertedLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetPauseTrigType(taskHandle, data)
    ccall((:DAQmxGetPauseTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetPauseTrigType(taskHandle, data)
    ccall((:DAQmxSetPauseTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetPauseTrigType(taskHandle)
    ccall((:DAQmxResetPauseTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetPauseTrigTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetPauseTrigTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetAnlgLvlPauseTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgLvlPauseTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigWhen(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigLvl(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigLvl, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigLvl(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigLvl, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigLvl(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigLvl, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigHyst(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigHyst, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigHyst(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigHyst, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigHyst(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigHyst, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgLvlPauseTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgLvlPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgLvlPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgLvlPauseTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinPauseTrigSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigWhen(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigTop(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigTop, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigTop(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigTop, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigTop(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigTop, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigBtm(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigBtm, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigBtm(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigBtm, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigBtm(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigBtm, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigCoupling(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigCoupling, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigCoupling(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigCoupling, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigCoupling(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigCoupling, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetAnlgWinPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetAnlgWinPauseTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetAnlgWinPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetAnlgWinPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetAnlgWinPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetAnlgWinPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetAnlgWinPauseTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetAnlgWinPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigLvlPauseTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigSrc(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxGetDigLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigLvlPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigWhen(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetDigLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigLvlPauseTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetDigLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigLvlPauseTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigLvlPauseTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetDigLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigLvlPauseTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigLvlPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetDigLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigLvlPauseTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetDigLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigLvlPauseTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetDigLvlPauseTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternPauseTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternPauseTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigPatternPauseTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternPauseTrigSrc(taskHandle)
    ccall((:DAQmxResetDigPatternPauseTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternPauseTrigPattern(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigPatternPauseTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigPatternPauseTrigPattern(taskHandle, data)
    ccall((:DAQmxSetDigPatternPauseTrigPattern, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigPatternPauseTrigPattern(taskHandle)
    ccall((:DAQmxResetDigPatternPauseTrigPattern, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigPatternPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxGetDigPatternPauseTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigPatternPauseTrigWhen(taskHandle, data)
    ccall((:DAQmxSetDigPatternPauseTrigWhen, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigPatternPauseTrigWhen(taskHandle)
    ccall((:DAQmxResetDigPatternPauseTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigType(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetArmStartTrigType(taskHandle, data)
    ccall((:DAQmxSetArmStartTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetArmStartTrigType(taskHandle)
    ccall((:DAQmxResetArmStartTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTerm(taskHandle, data, bufferSize)
    ccall((:DAQmxGetArmStartTerm, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetDigEdgeArmStartTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeArmStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeArmStartTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigEdge(taskHandle, data)
    ccall((:DAQmxGetDigEdgeArmStartTrigEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigEdgeArmStartTrigEdge(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigEdge(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeArmStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeArmStartTrigDigFltrEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigDigFltrEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigDigFltrEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxGetDigEdgeArmStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeArmStartTrigDigFltrMinPulseWidth(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigDigFltrMinPulseWidth(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigDigFltrMinPulseWidth, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxGetDigEdgeArmStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseRate(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseRate(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigDigFltrTimebaseRate, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeArmStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxGetDigEdgeArmStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetDigEdgeArmStartTrigDigSyncEnable(taskHandle, data)
    ccall((:DAQmxSetDigEdgeArmStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetDigEdgeArmStartTrigDigSyncEnable(taskHandle)
    ccall((:DAQmxResetDigEdgeArmStartTrigDigSyncEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigTrigWhen(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigTrigWhen, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxSetArmStartTrigTrigWhen(taskHandle, data)
    ccall((:DAQmxSetArmStartTrigTrigWhen, nidaqmx), int32, (TaskHandle, CVIAbsoluteTime), taskHandle, data)
end

function DAQmxResetArmStartTrigTrigWhen(taskHandle)
    ccall((:DAQmxResetArmStartTrigTrigWhen, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigTimescale(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetArmStartTrigTimescale(taskHandle, data)
    ccall((:DAQmxSetArmStartTrigTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetArmStartTrigTimescale(taskHandle)
    ccall((:DAQmxResetArmStartTrigTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetArmStartTrigTimestampEnable(taskHandle, data)
    ccall((:DAQmxSetArmStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetArmStartTrigTimestampEnable(taskHandle)
    ccall((:DAQmxResetArmStartTrigTimestampEnable, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetArmStartTrigTimestampTimescale(taskHandle, data)
    ccall((:DAQmxSetArmStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetArmStartTrigTimestampTimescale(taskHandle)
    ccall((:DAQmxResetArmStartTrigTimestampTimescale, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetArmStartTrigTimestampVal(taskHandle, data)
    ccall((:DAQmxGetArmStartTrigTimestampVal, nidaqmx), int32, (TaskHandle, Ptr{CVIAbsoluteTime}), taskHandle, data)
end

function DAQmxGetTriggerSyncType(taskHandle, data)
    ccall((:DAQmxGetTriggerSyncType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetTriggerSyncType(taskHandle, data)
    ccall((:DAQmxSetTriggerSyncType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetTriggerSyncType(taskHandle)
    ccall((:DAQmxResetTriggerSyncType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWatchdogTimeout(taskHandle, data)
    ccall((:DAQmxGetWatchdogTimeout, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetWatchdogTimeout(taskHandle, data)
    ccall((:DAQmxSetWatchdogTimeout, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetWatchdogTimeout(taskHandle)
    ccall((:DAQmxResetWatchdogTimeout, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWatchdogExpirTrigType(taskHandle, data)
    ccall((:DAQmxGetWatchdogExpirTrigType, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetWatchdogExpirTrigType(taskHandle, data)
    ccall((:DAQmxSetWatchdogExpirTrigType, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetWatchdogExpirTrigType(taskHandle)
    ccall((:DAQmxResetWatchdogExpirTrigType, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWatchdogExpirTrigTrigOnNetworkConnLoss(taskHandle, data)
    ccall((:DAQmxGetWatchdogExpirTrigTrigOnNetworkConnLoss, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxSetWatchdogExpirTrigTrigOnNetworkConnLoss(taskHandle, data)
    ccall((:DAQmxSetWatchdogExpirTrigTrigOnNetworkConnLoss, nidaqmx), int32, (TaskHandle, bool32), taskHandle, data)
end

function DAQmxResetWatchdogExpirTrigTrigOnNetworkConnLoss(taskHandle)
    ccall((:DAQmxResetWatchdogExpirTrigTrigOnNetworkConnLoss, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeWatchdogExpirTrigSrc(taskHandle, data, bufferSize)
    ccall((:DAQmxGetDigEdgeWatchdogExpirTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxSetDigEdgeWatchdogExpirTrigSrc(taskHandle, data)
    ccall((:DAQmxSetDigEdgeWatchdogExpirTrigSrc, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, data)
end

function DAQmxResetDigEdgeWatchdogExpirTrigSrc(taskHandle)
    ccall((:DAQmxResetDigEdgeWatchdogExpirTrigSrc, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetDigEdgeWatchdogExpirTrigEdge(taskHandle, data)
    ccall((:DAQmxGetDigEdgeWatchdogExpirTrigEdge, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetDigEdgeWatchdogExpirTrigEdge(taskHandle, data)
    ccall((:DAQmxSetDigEdgeWatchdogExpirTrigEdge, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetDigEdgeWatchdogExpirTrigEdge(taskHandle)
    ccall((:DAQmxResetDigEdgeWatchdogExpirTrigEdge, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWatchdogDOExpirState(taskHandle, lines, data)
    ccall((:DAQmxGetWatchdogDOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, lines, data)
end

function DAQmxSetWatchdogDOExpirState(taskHandle, lines, data)
    ccall((:DAQmxSetWatchdogDOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, lines, data)
end

function DAQmxResetWatchdogDOExpirState(taskHandle, lines)
    ccall((:DAQmxResetWatchdogDOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, lines)
end

function DAQmxGetWatchdogAOOutputType(taskHandle, lines, data)
    ccall((:DAQmxGetWatchdogAOOutputType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, lines, data)
end

function DAQmxSetWatchdogAOOutputType(taskHandle, lines, data)
    ccall((:DAQmxSetWatchdogAOOutputType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, lines, data)
end

function DAQmxResetWatchdogAOOutputType(taskHandle, lines)
    ccall((:DAQmxResetWatchdogAOOutputType, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, lines)
end

function DAQmxGetWatchdogAOExpirState(taskHandle, lines, data)
    ccall((:DAQmxGetWatchdogAOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{float64}), taskHandle, lines, data)
end

function DAQmxSetWatchdogAOExpirState(taskHandle, lines, data)
    ccall((:DAQmxSetWatchdogAOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, float64), taskHandle, lines, data)
end

function DAQmxResetWatchdogAOExpirState(taskHandle, lines)
    ccall((:DAQmxResetWatchdogAOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, lines)
end

function DAQmxGetWatchdogCOExpirState(taskHandle, lines, data)
    ccall((:DAQmxGetWatchdogCOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, Ptr{int32}), taskHandle, lines, data)
end

function DAQmxSetWatchdogCOExpirState(taskHandle, lines, data)
    ccall((:DAQmxSetWatchdogCOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, int32), taskHandle, lines, data)
end

function DAQmxResetWatchdogCOExpirState(taskHandle, lines)
    ccall((:DAQmxResetWatchdogCOExpirState, nidaqmx), int32, (TaskHandle, Ptr{Cchar}), taskHandle, lines)
end

function DAQmxGetWatchdogHasExpired(taskHandle, data)
    ccall((:DAQmxGetWatchdogHasExpired, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteRelativeTo(taskHandle, data)
    ccall((:DAQmxGetWriteRelativeTo, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetWriteRelativeTo(taskHandle, data)
    ccall((:DAQmxSetWriteRelativeTo, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetWriteRelativeTo(taskHandle)
    ccall((:DAQmxResetWriteRelativeTo, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWriteOffset(taskHandle, data)
    ccall((:DAQmxGetWriteOffset, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetWriteOffset(taskHandle, data)
    ccall((:DAQmxSetWriteOffset, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetWriteOffset(taskHandle)
    ccall((:DAQmxResetWriteOffset, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWriteRegenMode(taskHandle, data)
    ccall((:DAQmxGetWriteRegenMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetWriteRegenMode(taskHandle, data)
    ccall((:DAQmxSetWriteRegenMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetWriteRegenMode(taskHandle)
    ccall((:DAQmxResetWriteRegenMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWriteCurrWritePos(taskHandle, data)
    ccall((:DAQmxGetWriteCurrWritePos, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxGetWriteOvercurrentChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteOvercurrentChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteOvercurrentChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteOvercurrentChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteOvertemperatureChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteOvertemperatureChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteOvertemperatureChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteOvertemperatureChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteExternalOvervoltageChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteExternalOvervoltageChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteExternalOvervoltageChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteExternalOvervoltageChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteOverloadedChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteOverloadedChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteOverloadedChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteOverloadedChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteOpenCurrentLoopChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteOpenCurrentLoopChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteOpenCurrentLoopChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteOpenCurrentLoopChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWritePowerSupplyFaultChansExist(taskHandle, data)
    ccall((:DAQmxGetWritePowerSupplyFaultChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWritePowerSupplyFaultChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWritePowerSupplyFaultChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteSyncUnlockedChansExist(taskHandle, data)
    ccall((:DAQmxGetWriteSyncUnlockedChansExist, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteSyncUnlockedChans(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteSyncUnlockedChans, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteSpaceAvail(taskHandle, data)
    ccall((:DAQmxGetWriteSpaceAvail, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetWriteTotalSampPerChanGenerated(taskHandle, data)
    ccall((:DAQmxGetWriteTotalSampPerChanGenerated, nidaqmx), int32, (TaskHandle, Ptr{uInt64}), taskHandle, data)
end

function DAQmxGetWriteAccessoryInsertionOrRemovalDetected(taskHandle, data)
    ccall((:DAQmxGetWriteAccessoryInsertionOrRemovalDetected, nidaqmx), int32, (TaskHandle, Ptr{bool32}), taskHandle, data)
end

function DAQmxGetWriteDevsWithInsertedOrRemovedAccessories(taskHandle, data, bufferSize)
    ccall((:DAQmxGetWriteDevsWithInsertedOrRemovedAccessories, nidaqmx), int32, (TaskHandle, Ptr{Cchar}, uInt32), taskHandle, data, bufferSize)
end

function DAQmxGetWriteRawDataWidth(taskHandle, data)
    ccall((:DAQmxGetWriteRawDataWidth, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetWriteNumChans(taskHandle, data)
    ccall((:DAQmxGetWriteNumChans, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetWriteWaitMode(taskHandle, data)
    ccall((:DAQmxGetWriteWaitMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetWriteWaitMode(taskHandle, data)
    ccall((:DAQmxSetWriteWaitMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetWriteWaitMode(taskHandle)
    ccall((:DAQmxResetWriteWaitMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWriteSleepTime(taskHandle, data)
    ccall((:DAQmxGetWriteSleepTime, nidaqmx), int32, (TaskHandle, Ptr{float64}), taskHandle, data)
end

function DAQmxSetWriteSleepTime(taskHandle, data)
    ccall((:DAQmxSetWriteSleepTime, nidaqmx), int32, (TaskHandle, float64), taskHandle, data)
end

function DAQmxResetWriteSleepTime(taskHandle)
    ccall((:DAQmxResetWriteSleepTime, nidaqmx), int32, (TaskHandle,), taskHandle)
end

function DAQmxGetWriteDigitalLinesBytesPerChan(taskHandle, data)
    ccall((:DAQmxGetWriteDigitalLinesBytesPerChan, nidaqmx), int32, (TaskHandle, Ptr{uInt32}), taskHandle, data)
end

function DAQmxGetSampClkTimingResponseMode(taskHandle, data)
    ccall((:DAQmxGetSampClkTimingResponseMode, nidaqmx), int32, (TaskHandle, Ptr{int32}), taskHandle, data)
end

function DAQmxSetSampClkTimingResponseMode(taskHandle, data)
    ccall((:DAQmxSetSampClkTimingResponseMode, nidaqmx), int32, (TaskHandle, int32), taskHandle, data)
end

function DAQmxResetSampClkTimingResponseMode(taskHandle)
    ccall((:DAQmxResetSampClkTimingResponseMode, nidaqmx), int32, (TaskHandle,), taskHandle)
end

#const CVICALLBACK = CVICDECL

const TRUE = Clong(1)

const FALSE = Clong(0)

const NULL = Clong(0)

const DAQmx_Buf_Input_BufSize = 0x186c

const DAQmx_Buf_Input_OnbrdBufSize = 0x230a

const DAQmx_Buf_Output_BufSize = 0x186d

const DAQmx_Buf_Output_OnbrdBufSize = 0x230b

const DAQmx_SelfCal_Supported = 0x1860

const DAQmx_SelfCal_LastTemp = 0x1864

const DAQmx_ExtCal_RecommendedInterval = 0x1868

const DAQmx_ExtCal_LastTemp = 0x1867

const DAQmx_Cal_UserDefinedInfo = 0x1861

const DAQmx_Cal_UserDefinedInfo_MaxSize = 0x191c

const DAQmx_Cal_DevTemp = 0x223b

const DAQmx_Cal_AccConnectionCount = 0x2feb

const DAQmx_Cal_RecommendedAccConnectionCountLimit = 0x2fec

const DAQmx_AI_Max = 0x17dd

const DAQmx_AI_Min = 0x17de

const DAQmx_AI_CustomScaleName = 0x17e0

const DAQmx_AI_MeasType = 0x0695

const DAQmx_AI_Voltage_Units = 0x1094

const DAQmx_AI_Voltage_dBRef = 0x29b0

const DAQmx_AI_Voltage_ACRMS_Units = 0x17e2

const DAQmx_AI_Temp_Units = 0x1033

const DAQmx_AI_Thrmcpl_Type = 0x1050

const DAQmx_AI_Thrmcpl_ScaleType = 0x29d0

const DAQmx_AI_Thrmcpl_CJCSrc = 0x1035

const DAQmx_AI_Thrmcpl_CJCVal = 0x1036

const DAQmx_AI_Thrmcpl_CJCChan = 0x1034

const DAQmx_AI_RTD_Type = 0x1032

const DAQmx_AI_RTD_R0 = 0x1030

const DAQmx_AI_RTD_A = 0x1010

const DAQmx_AI_RTD_B = 0x1011

const DAQmx_AI_RTD_C = 0x1013

const DAQmx_AI_Thrmstr_A = 0x18c9

const DAQmx_AI_Thrmstr_B = 0x18cb

const DAQmx_AI_Thrmstr_C = 0x18ca

const DAQmx_AI_Thrmstr_R1 = 0x1061

const DAQmx_AI_ForceReadFromChan = 0x18f8

const DAQmx_AI_Current_Units = 0x0701

const DAQmx_AI_Current_ACRMS_Units = 0x17e3

const DAQmx_AI_Strain_Units = 0x0981

const DAQmx_AI_StrainGage_ForceReadFromChan = 0x2ffa

const DAQmx_AI_StrainGage_GageFactor = 0x0994

const DAQmx_AI_StrainGage_PoissonRatio = 0x0998

const DAQmx_AI_StrainGage_Cfg = 0x0982

const DAQmx_AI_RosetteStrainGage_RosetteType = 0x2ffe

const DAQmx_AI_RosetteStrainGage_Orientation = 0x2ffc

const DAQmx_AI_RosetteStrainGage_StrainChans = 0x2ffb

const DAQmx_AI_RosetteStrainGage_RosetteMeasType = 0x2ffd

const DAQmx_AI_Resistance_Units = 0x0955

const DAQmx_AI_Freq_Units = 0x0806

const DAQmx_AI_Freq_ThreshVoltage = 0x0815

const DAQmx_AI_Freq_Hyst = 0x0814

const DAQmx_AI_LVDT_Units = 0x0910

const DAQmx_AI_LVDT_Sensitivity = 0x0939

const DAQmx_AI_LVDT_SensitivityUnits = 0x219a

const DAQmx_AI_RVDT_Units = 0x0877

const DAQmx_AI_RVDT_Sensitivity = 0x0903

const DAQmx_AI_RVDT_SensitivityUnits = 0x219b

const DAQmx_AI_EddyCurrentProxProbe_Units = 0x2ac0

const DAQmx_AI_EddyCurrentProxProbe_Sensitivity = 0x2abe

const DAQmx_AI_EddyCurrentProxProbe_SensitivityUnits = 0x2abf

const DAQmx_AI_SoundPressure_MaxSoundPressureLvl = 0x223a

const DAQmx_AI_SoundPressure_Units = 0x1528

const DAQmx_AI_SoundPressure_dBRef = 0x29b1

const DAQmx_AI_Microphone_Sensitivity = 0x1536

const DAQmx_AI_Accel_Units = 0x0673

const DAQmx_AI_Accel_dBRef = 0x29b2

const DAQmx_AI_Accel_4WireDCVoltage_Sensitivity = 0x3115

const DAQmx_AI_Accel_4WireDCVoltage_SensitivityUnits = 0x3116

const DAQmx_AI_Accel_Sensitivity = 0x0692

const DAQmx_AI_Accel_SensitivityUnits = 0x219c

const DAQmx_AI_Accel_Charge_Sensitivity = 0x3113

const DAQmx_AI_Accel_Charge_SensitivityUnits = 0x3114

const DAQmx_AI_Velocity_Units = 0x2ff4

const DAQmx_AI_Velocity_IEPESensor_dBRef = 0x2ff5

const DAQmx_AI_Velocity_IEPESensor_Sensitivity = 0x2ff6

const DAQmx_AI_Velocity_IEPESensor_SensitivityUnits = 0x2ff7

const DAQmx_AI_Force_Units = 0x2f75

const DAQmx_AI_Force_IEPESensor_Sensitivity = 0x2f81

const DAQmx_AI_Force_IEPESensor_SensitivityUnits = 0x2f82

const DAQmx_AI_Pressure_Units = 0x2f76

const DAQmx_AI_Torque_Units = 0x2f77

const DAQmx_AI_Bridge_Units = 0x2f92

const DAQmx_AI_Bridge_ElectricalUnits = 0x2f87

const DAQmx_AI_Bridge_PhysicalUnits = 0x2f88

const DAQmx_AI_Bridge_ScaleType = 0x2f89

const DAQmx_AI_Bridge_TwoPointLin_First_ElectricalVal = 0x2f8a

const DAQmx_AI_Bridge_TwoPointLin_First_PhysicalVal = 0x2f8b

const DAQmx_AI_Bridge_TwoPointLin_Second_ElectricalVal = 0x2f8c

const DAQmx_AI_Bridge_TwoPointLin_Second_PhysicalVal = 0x2f8d

const DAQmx_AI_Bridge_Table_ElectricalVals = 0x2f8e

const DAQmx_AI_Bridge_Table_PhysicalVals = 0x2f8f

const DAQmx_AI_Bridge_Poly_ForwardCoeff = 0x2f90

const DAQmx_AI_Bridge_Poly_ReverseCoeff = 0x2f91

const DAQmx_AI_Charge_Units = 0x3112

const DAQmx_AI_Is_TEDS = 0x2983

const DAQmx_AI_TEDS_Units = 0x21e0

const DAQmx_AI_Coupling = 0x0064

const DAQmx_AI_Impedance = 0x0062

const DAQmx_AI_TermCfg = 0x1097

const DAQmx_AI_InputSrc = 0x2198

const DAQmx_AI_ResistanceCfg = 0x1881

const DAQmx_AI_LeadWireResistance = 0x17ee

const DAQmx_AI_Bridge_Cfg = 0x0087

const DAQmx_AI_Bridge_NomResistance = 0x17ec

const DAQmx_AI_Bridge_InitialVoltage = 0x17ed

const DAQmx_AI_Bridge_InitialRatio = 0x2f86

const DAQmx_AI_Bridge_ShuntCal_Enable = 0x0094

const DAQmx_AI_Bridge_ShuntCal_Select = 0x21d5

const DAQmx_AI_Bridge_ShuntCal_ShuntCalASrc = 0x30ca

const DAQmx_AI_Bridge_ShuntCal_GainAdjust = 0x193f

const DAQmx_AI_Bridge_ShuntCal_ShuntCalAResistance = 0x2f78

const DAQmx_AI_Bridge_ShuntCal_ShuntCalAActualResistance = 0x2f79

const DAQmx_AI_Bridge_ShuntCal_ShuntCalBResistance = 0x2f7a

const DAQmx_AI_Bridge_ShuntCal_ShuntCalBActualResistance = 0x2f7b

const DAQmx_AI_Bridge_Balance_CoarsePot = 0x17f1

const DAQmx_AI_Bridge_Balance_FinePot = 0x18f4

const DAQmx_AI_CurrentShunt_Loc = 0x17f2

const DAQmx_AI_CurrentShunt_Resistance = 0x17f3

const DAQmx_AI_Excit_Sense = 0x30fd

const DAQmx_AI_Excit_Src = 0x17f4

const DAQmx_AI_Excit_Val = 0x17f5

const DAQmx_AI_Excit_UseForScaling = 0x17fc

const DAQmx_AI_Excit_UseMultiplexed = 0x2180

const DAQmx_AI_Excit_ActualVal = 0x1883

const DAQmx_AI_Excit_DCorAC = 0x17fb

const DAQmx_AI_Excit_VoltageOrCurrent = 0x17f6

const DAQmx_AI_Excit_IdleOutputBehavior = 0x30b8

const DAQmx_AI_ACExcit_Freq = 0x0101

const DAQmx_AI_ACExcit_SyncEnable = 0x0102

const DAQmx_AI_ACExcit_WireMode = 0x18cd

const DAQmx_AI_SensorPower_Voltage = 0x3169

const DAQmx_AI_SensorPower_Cfg = 0x316a

const DAQmx_AI_SensorPower_Type = 0x316b

const DAQmx_AI_OpenThrmcplDetectEnable = 0x2f72

const DAQmx_AI_Thrmcpl_LeadOffsetVoltage = 0x2fb8

const DAQmx_AI_Atten = 0x1801

const DAQmx_AI_ProbeAtten = 0x2a88

const DAQmx_AI_Lowpass_Enable = 0x1802

const DAQmx_AI_Lowpass_CutoffFreq = 0x1803

const DAQmx_AI_Lowpass_SwitchCap_ClkSrc = 0x1884

const DAQmx_AI_Lowpass_SwitchCap_ExtClkFreq = 0x1885

const DAQmx_AI_Lowpass_SwitchCap_ExtClkDiv = 0x1886

const DAQmx_AI_Lowpass_SwitchCap_OutClkDiv = 0x1887

const DAQmx_AI_DigFltr_Enable = 0x30bd

const DAQmx_AI_DigFltr_Type = 0x30be

const DAQmx_AI_DigFltr_Response = 0x30bf

const DAQmx_AI_DigFltr_Order = 0x30c0

const DAQmx_AI_DigFltr_Lowpass_CutoffFreq = 0x30c1

const DAQmx_AI_DigFltr_Highpass_CutoffFreq = 0x30c2

const DAQmx_AI_DigFltr_Bandpass_CenterFreq = 0x30c3

const DAQmx_AI_DigFltr_Bandpass_Width = 0x30c4

const DAQmx_AI_DigFltr_Notch_CenterFreq = 0x30c5

const DAQmx_AI_DigFltr_Notch_Width = 0x30c6

const DAQmx_AI_DigFltr_Coeff = 0x30c7

const DAQmx_AI_Filter_Enable = 0x3173

const DAQmx_AI_Filter_Freq = 0x3174

const DAQmx_AI_Filter_Response = 0x3175

const DAQmx_AI_Filter_Order = 0x3176

const DAQmx_AI_FilterDelay = 0x2fed

const DAQmx_AI_FilterDelayUnits = 0x3071

const DAQmx_AI_RemoveFilterDelay = 0x2fbd

const DAQmx_AI_FilterDelayAdjustment = 0x3074

const DAQmx_AI_AveragingWinSize = 0x2fee

const DAQmx_AI_ResolutionUnits = 0x1764

const DAQmx_AI_Resolution = 0x1765

const DAQmx_AI_RawSampSize = 0x22da

const DAQmx_AI_RawSampJustification = 0x0050

const DAQmx_AI_ADCTimingMode = 0x29f9

const DAQmx_AI_ADCCustomTimingMode = 0x2f6b

const DAQmx_AI_Dither_Enable = 0x0068

const DAQmx_AI_ChanCal_HasValidCalInfo = 0x2297

const DAQmx_AI_ChanCal_EnableCal = 0x2298

const DAQmx_AI_ChanCal_ApplyCalIfExp = 0x2299

const DAQmx_AI_ChanCal_ScaleType = 0x229c

const DAQmx_AI_ChanCal_Table_PreScaledVals = 0x229d

const DAQmx_AI_ChanCal_Table_ScaledVals = 0x229e

const DAQmx_AI_ChanCal_Poly_ForwardCoeff = 0x229f

const DAQmx_AI_ChanCal_Poly_ReverseCoeff = 0x22a0

const DAQmx_AI_ChanCal_OperatorName = 0x22a3

const DAQmx_AI_ChanCal_Desc = 0x22a4

const DAQmx_AI_ChanCal_Verif_RefVals = 0x22a1

const DAQmx_AI_ChanCal_Verif_AcqVals = 0x22a2

const DAQmx_AI_Rng_High = 0x1815

const DAQmx_AI_Rng_Low = 0x1816

const DAQmx_AI_DCOffset = 0x2a89

const DAQmx_AI_Gain = 0x1818

const DAQmx_AI_SampAndHold_Enable = 0x181a

const DAQmx_AI_AutoZeroMode = 0x1760

const DAQmx_AI_ChopEnable = 0x3143

const DAQmx_AI_DataXferMaxRate = 0x3117

const DAQmx_AI_DataXferMech = 0x1821

const DAQmx_AI_DataXferReqCond = 0x188b

const DAQmx_AI_DataXferCustomThreshold = 0x230c

const DAQmx_AI_UsbXferReqSize = 0x2a8e

const DAQmx_AI_UsbXferReqCount = 0x3000

const DAQmx_AI_MemMapEnable = 0x188c

const DAQmx_AI_RawDataCompressionType = 0x22d8

const DAQmx_AI_LossyLSBRemoval_CompressedSampSize = 0x22d9

const DAQmx_AI_DevScalingCoeff = 0x1930

const DAQmx_AI_EnhancedAliasRejectionEnable = 0x2294

const DAQmx_AI_OpenChanDetectEnable = 0x30ff

const DAQmx_AI_InputLimitsFaultDetect_UpperLimit = 0x318c

const DAQmx_AI_InputLimitsFaultDetect_LowerLimit = 0x318d

const DAQmx_AI_InputLimitsFaultDetectEnable = 0x318e

const DAQmx_AI_PowerSupplyFaultDetectEnable = 0x3191

const DAQmx_AI_OvercurrentDetectEnable = 0x3194

const DAQmx_AO_Max = 0x1186

const DAQmx_AO_Min = 0x1187

const DAQmx_AO_CustomScaleName = 0x1188

const DAQmx_AO_OutputType = 0x1108

const DAQmx_AO_Voltage_Units = 0x1184

const DAQmx_AO_Voltage_CurrentLimit = 0x2a1d

const DAQmx_AO_Current_Units = 0x1109

const DAQmx_AO_FuncGen_Type = 0x2a18

const DAQmx_AO_FuncGen_Freq = 0x2a19

const DAQmx_AO_FuncGen_Amplitude = 0x2a1a

const DAQmx_AO_FuncGen_Offset = 0x2a1b

const DAQmx_AO_FuncGen_StartPhase = 0x31c4

const DAQmx_AO_FuncGen_Square_DutyCycle = 0x2a1c

const DAQmx_AO_FuncGen_ModulationType = 0x2a22

const DAQmx_AO_FuncGen_FMDeviation = 0x2a23

const DAQmx_AO_OutputImpedance = 0x1490

const DAQmx_AO_LoadImpedance = 0x0121

const DAQmx_AO_IdleOutputBehavior = 0x2240

const DAQmx_AO_TermCfg = 0x188e

const DAQmx_AO_Common_Mode_Offset = 0x31cc

const DAQmx_AO_ResolutionUnits = 0x182b

const DAQmx_AO_Resolution = 0x182c

const DAQmx_AO_DAC_Rng_High = 0x182e

const DAQmx_AO_DAC_Rng_Low = 0x182d

const DAQmx_AO_DAC_Ref_ConnToGnd = 0x0130

const DAQmx_AO_DAC_Ref_AllowConnToGnd = 0x1830

const DAQmx_AO_DAC_Ref_Src = 0x0132

const DAQmx_AO_DAC_Ref_ExtSrc = 0x2252

const DAQmx_AO_DAC_Ref_Val = 0x1832

const DAQmx_AO_DAC_Offset_Src = 0x2253

const DAQmx_AO_DAC_Offset_ExtSrc = 0x2254

const DAQmx_AO_DAC_Offset_Val = 0x2255

const DAQmx_AO_ReglitchEnable = 0x0133

const DAQmx_AO_FilterDelay = 0x3075

const DAQmx_AO_FilterDelayUnits = 0x3076

const DAQmx_AO_FilterDelayAdjustment = 0x3072

const DAQmx_AO_Gain = 0x0118

const DAQmx_AO_UseOnlyOnBrdMem = 0x183a

const DAQmx_AO_DataXferMech = 0x0134

const DAQmx_AO_DataXferReqCond = 0x183c

const DAQmx_AO_UsbXferReqSize = 0x2a8f

const DAQmx_AO_UsbXferReqCount = 0x3001

const DAQmx_AO_MemMapEnable = 0x188f

const DAQmx_AO_DevScalingCoeff = 0x1931

const DAQmx_AO_EnhancedImageRejectionEnable = 0x2241

const DAQmx_DI_InvertLines = 0x0793

const DAQmx_DI_NumLines = 0x2178

const DAQmx_DI_DigFltr_Enable = 0x21d6

const DAQmx_DI_DigFltr_MinPulseWidth = 0x21d7

const DAQmx_DI_DigFltr_EnableBusMode = 0x2efe

const DAQmx_DI_DigFltr_TimebaseSrc = 0x2ed4

const DAQmx_DI_DigFltr_TimebaseRate = 0x2ed5

const DAQmx_DI_DigSync_Enable = 0x2ed6

const DAQmx_DI_Tristate = 0x1890

const DAQmx_DI_LogicFamily = 0x296d

const DAQmx_DI_DataXferMech = 0x2263

const DAQmx_DI_DataXferReqCond = 0x2264

const DAQmx_DI_UsbXferReqSize = 0x2a90

const DAQmx_DI_UsbXferReqCount = 0x3002

const DAQmx_DI_MemMapEnable = 0x296a

const DAQmx_DI_AcquireOn = 0x2966

const DAQmx_DO_OutputDriveType = 0x1137

const DAQmx_DO_InvertLines = 0x1133

const DAQmx_DO_NumLines = 0x2179

const DAQmx_DO_Tristate = 0x18f3

const DAQmx_DO_LineStates_StartState = 0x2972

const DAQmx_DO_LineStates_PausedState = 0x2967

const DAQmx_DO_LineStates_DoneState = 0x2968

const DAQmx_DO_LogicFamily = 0x296e

const DAQmx_DO_Overcurrent_Limit = 0x2a85

const DAQmx_DO_Overcurrent_AutoReenable = 0x2a86

const DAQmx_DO_Overcurrent_ReenablePeriod = 0x2a87

const DAQmx_DO_UseOnlyOnBrdMem = 0x2265

const DAQmx_DO_DataXferMech = 0x2266

const DAQmx_DO_DataXferReqCond = 0x2267

const DAQmx_DO_UsbXferReqSize = 0x2a91

const DAQmx_DO_UsbXferReqCount = 0x3003

const DAQmx_DO_MemMapEnable = 0x296b

const DAQmx_DO_GenerateOn = 0x2969

const DAQmx_CI_Max = 0x189c

const DAQmx_CI_Min = 0x189d

const DAQmx_CI_CustomScaleName = 0x189e

const DAQmx_CI_MeasType = 0x18a0

const DAQmx_CI_Freq_Units = 0x18a1

const DAQmx_CI_Freq_Term = 0x18a2

const DAQmx_CI_Freq_TermCfg = 0x3097

const DAQmx_CI_Freq_LogicLvlBehavior = 0x3098

const DAQmx_CI_Freq_ThreshVoltage = 0x31ab

const DAQmx_CI_Freq_Hyst = 0x31ac

const DAQmx_CI_Freq_DigFltr_Enable = 0x21e7

const DAQmx_CI_Freq_DigFltr_MinPulseWidth = 0x21e8

const DAQmx_CI_Freq_DigFltr_TimebaseSrc = 0x21e9

const DAQmx_CI_Freq_DigFltr_TimebaseRate = 0x21ea

const DAQmx_CI_Freq_DigSync_Enable = 0x21eb

const DAQmx_CI_Freq_StartingEdge = 0x0799

const DAQmx_CI_Freq_MeasMeth = 0x0144

const DAQmx_CI_Freq_EnableAveraging = 0x2ed0

const DAQmx_CI_Freq_MeasTime = 0x0145

const DAQmx_CI_Freq_Div = 0x0147

const DAQmx_CI_Period_Units = 0x18a3

const DAQmx_CI_Period_Term = 0x18a4

const DAQmx_CI_Period_TermCfg = 0x3099

const DAQmx_CI_Period_LogicLvlBehavior = 0x309a

const DAQmx_CI_Period_ThreshVoltage = 0x31ad

const DAQmx_CI_Period_Hyst = 0x31ae

const DAQmx_CI_Period_DigFltr_Enable = 0x21ec

const DAQmx_CI_Period_DigFltr_MinPulseWidth = 0x21ed

const DAQmx_CI_Period_DigFltr_TimebaseSrc = 0x21ee

const DAQmx_CI_Period_DigFltr_TimebaseRate = 0x21ef

const DAQmx_CI_Period_DigSync_Enable = 0x21f0

const DAQmx_CI_Period_StartingEdge = 0x0852

const DAQmx_CI_Period_MeasMeth = 0x192c

const DAQmx_CI_Period_EnableAveraging = 0x2ed1

const DAQmx_CI_Period_MeasTime = 0x192d

const DAQmx_CI_Period_Div = 0x192e

const DAQmx_CI_CountEdges_Term = 0x18c7

const DAQmx_CI_CountEdges_TermCfg = 0x309b

const DAQmx_CI_CountEdges_LogicLvlBehavior = 0x309c

const DAQmx_CI_CountEdges_ThreshVoltage = 0x31af

const DAQmx_CI_CountEdges_Hyst = 0x31b0

const DAQmx_CI_CountEdges_DigFltr_Enable = 0x21f6

const DAQmx_CI_CountEdges_DigFltr_MinPulseWidth = 0x21f7

const DAQmx_CI_CountEdges_DigFltr_TimebaseSrc = 0x21f8

const DAQmx_CI_CountEdges_DigFltr_TimebaseRate = 0x21f9

const DAQmx_CI_CountEdges_DigSync_Enable = 0x21fa

const DAQmx_CI_CountEdges_Dir = 0x0696

const DAQmx_CI_CountEdges_DirTerm = 0x21e1

const DAQmx_CI_CountEdges_CountDir_TermCfg = 0x309d

const DAQmx_CI_CountEdges_CountDir_LogicLvlBehavior = 0x309e

const DAQmx_CI_CountEdges_CountDir_ThreshVoltage = 0x31b1

const DAQmx_CI_CountEdges_CountDir_Hyst = 0x31b2

const DAQmx_CI_CountEdges_CountDir_DigFltr_Enable = 0x21f1

const DAQmx_CI_CountEdges_CountDir_DigFltr_MinPulseWidth = 0x21f2

const DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseSrc = 0x21f3

const DAQmx_CI_CountEdges_CountDir_DigFltr_TimebaseRate = 0x21f4

const DAQmx_CI_CountEdges_CountDir_DigSync_Enable = 0x21f5

const DAQmx_CI_CountEdges_InitialCnt = 0x0698

const DAQmx_CI_CountEdges_ActiveEdge = 0x0697

const DAQmx_CI_CountEdges_CountReset_Enable = 0x2faf

const DAQmx_CI_CountEdges_CountReset_ResetCount = 0x2fb0

const DAQmx_CI_CountEdges_CountReset_Term = 0x2fb1

const DAQmx_CI_CountEdges_CountReset_TermCfg = 0x309f

const DAQmx_CI_CountEdges_CountReset_LogicLvlBehavior = 0x30a0

const DAQmx_CI_CountEdges_CountReset_ThreshVoltage = 0x31b3

const DAQmx_CI_CountEdges_CountReset_Hyst = 0x31b4

const DAQmx_CI_CountEdges_CountReset_DigFltr_Enable = 0x2fb3

const DAQmx_CI_CountEdges_CountReset_DigFltr_MinPulseWidth = 0x2fb4

const DAQmx_CI_CountEdges_CountReset_DigFltr_TimebaseSrc = 0x2fb5

const DAQmx_CI_CountEdges_CountReset_DigFltr_TimebaseRate = 0x2fb6

const DAQmx_CI_CountEdges_CountReset_DigSync_Enable = 0x2fb7

const DAQmx_CI_CountEdges_CountReset_ActiveEdge = 0x2fb2

const DAQmx_CI_CountEdges_Gate_Enable = 0x30ed

const DAQmx_CI_CountEdges_Gate_Term = 0x30ee

const DAQmx_CI_CountEdges_Gate_TermCfg = 0x30ef

const DAQmx_CI_CountEdges_Gate_LogicLvlBehavior = 0x30f0

const DAQmx_CI_CountEdges_Gate_ThreshVoltage = 0x31b5

const DAQmx_CI_CountEdges_Gate_Hyst = 0x31b6

const DAQmx_CI_CountEdges_Gate_DigFltrEnable = 0x30f1

const DAQmx_CI_CountEdges_Gate_DigFltrMinPulseWidth = 0x30f2

const DAQmx_CI_CountEdges_Gate_DigFltrTimebaseSrc = 0x30f3

const DAQmx_CI_CountEdges_Gate_DigFltrTimebaseRate = 0x30f4

const DAQmx_CI_CountEdges_GateWhen = 0x30f5

const DAQmx_CI_DutyCycle_Term = 0x308d

const DAQmx_CI_DutyCycle_TermCfg = 0x30a1

const DAQmx_CI_DutyCycle_LogicLvlBehavior = 0x30a2

const DAQmx_CI_DutyCycle_DigFltr_Enable = 0x308e

const DAQmx_CI_DutyCycle_DigFltr_MinPulseWidth = 0x308f

const DAQmx_CI_DutyCycle_DigFltr_TimebaseSrc = 0x3090

const DAQmx_CI_DutyCycle_DigFltr_TimebaseRate = 0x3091

const DAQmx_CI_DutyCycle_StartingEdge = 0x3092

const DAQmx_CI_AngEncoder_Units = 0x18a6

const DAQmx_CI_AngEncoder_PulsesPerRev = 0x0875

const DAQmx_CI_AngEncoder_InitialAngle = 0x0881

const DAQmx_CI_LinEncoder_Units = 0x18a9

const DAQmx_CI_LinEncoder_DistPerPulse = 0x0911

const DAQmx_CI_LinEncoder_InitialPos = 0x0915

const DAQmx_CI_Encoder_DecodingType = 0x21e6

const DAQmx_CI_Encoder_AInputTerm = 0x219d

const DAQmx_CI_Encoder_AInputTermCfg = 0x30a3

const DAQmx_CI_Encoder_AInputLogicLvlBehavior = 0x30a4

const DAQmx_CI_Encoder_AInput_DigFltr_Enable = 0x21fb

const DAQmx_CI_Encoder_AInput_DigFltr_MinPulseWidth = 0x21fc

const DAQmx_CI_Encoder_AInput_DigFltr_TimebaseSrc = 0x21fd

const DAQmx_CI_Encoder_AInput_DigFltr_TimebaseRate = 0x21fe

const DAQmx_CI_Encoder_AInput_DigSync_Enable = 0x21ff

const DAQmx_CI_Encoder_BInputTerm = 0x219e

const DAQmx_CI_Encoder_BInputTermCfg = 0x30a5

const DAQmx_CI_Encoder_BInputLogicLvlBehavior = 0x30a6

const DAQmx_CI_Encoder_BInput_DigFltr_Enable = 0x2200

const DAQmx_CI_Encoder_BInput_DigFltr_MinPulseWidth = 0x2201

const DAQmx_CI_Encoder_BInput_DigFltr_TimebaseSrc = 0x2202

const DAQmx_CI_Encoder_BInput_DigFltr_TimebaseRate = 0x2203

const DAQmx_CI_Encoder_BInput_DigSync_Enable = 0x2204

const DAQmx_CI_Encoder_ZInputTerm = 0x219f

const DAQmx_CI_Encoder_ZInputTermCfg = 0x30a7

const DAQmx_CI_Encoder_ZInputLogicLvlBehavior = 0x30a8

const DAQmx_CI_Encoder_ZInput_DigFltr_Enable = 0x2205

const DAQmx_CI_Encoder_ZInput_DigFltr_MinPulseWidth = 0x2206

const DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseSrc = 0x2207

const DAQmx_CI_Encoder_ZInput_DigFltr_TimebaseRate = 0x2208

const DAQmx_CI_Encoder_ZInput_DigSync_Enable = 0x2209

const DAQmx_CI_Encoder_ZIndexEnable = 0x0890

const DAQmx_CI_Encoder_ZIndexVal = 0x0888

const DAQmx_CI_Encoder_ZIndexPhase = 0x0889

const DAQmx_CI_PulseWidth_Units = 0x0823

const DAQmx_CI_PulseWidth_Term = 0x18aa

const DAQmx_CI_PulseWidth_TermCfg = 0x30a9

const DAQmx_CI_PulseWidth_LogicLvlBehavior = 0x30aa

const DAQmx_CI_PulseWidth_DigFltr_Enable = 0x220a

const DAQmx_CI_PulseWidth_DigFltr_MinPulseWidth = 0x220b

const DAQmx_CI_PulseWidth_DigFltr_TimebaseSrc = 0x220c

const DAQmx_CI_PulseWidth_DigFltr_TimebaseRate = 0x220d

const DAQmx_CI_PulseWidth_DigSync_Enable = 0x220e

const DAQmx_CI_PulseWidth_StartingEdge = 0x0825

const DAQmx_CI_Timestamp_Units = 0x22b3

const DAQmx_CI_Timestamp_InitialSeconds = 0x22b4

const DAQmx_CI_GPS_SyncMethod = 0x1092

const DAQmx_CI_GPS_SyncSrc = 0x1093

const DAQmx_CI_Velocity_AngEncoder_Units = 0x30d8

const DAQmx_CI_Velocity_AngEncoder_PulsesPerRev = 0x30d9

const DAQmx_CI_Velocity_LinEncoder_Units = 0x30da

const DAQmx_CI_Velocity_LinEncoder_DistPerPulse = 0x30db

const DAQmx_CI_Velocity_Encoder_DecodingType = 0x30dc

const DAQmx_CI_Velocity_Encoder_AInputTerm = 0x30dd

const DAQmx_CI_Velocity_Encoder_AInputTermCfg = 0x30de

const DAQmx_CI_Velocity_Encoder_AInputLogicLvlBehavior = 0x30df

const DAQmx_CI_Velocity_Encoder_AInputDigFltr_Enable = 0x30e0

const DAQmx_CI_Velocity_Encoder_AInputDigFltr_MinPulseWidth = 0x30e1

const DAQmx_CI_Velocity_Encoder_AInputDigFltr_TimebaseSrc = 0x30e2

const DAQmx_CI_Velocity_Encoder_AInputDigFltr_TimebaseRate = 0x30e3

const DAQmx_CI_Velocity_Encoder_BInputTerm = 0x30e4

const DAQmx_CI_Velocity_Encoder_BInputTermCfg = 0x30e5

const DAQmx_CI_Velocity_Encoder_BInputLogicLvlBehavior = 0x30e6

const DAQmx_CI_Velocity_Encoder_BInputDigFltr_Enable = 0x30e7

const DAQmx_CI_Velocity_Encoder_BInputDigFltr_MinPulseWidth = 0x30e8

const DAQmx_CI_Velocity_Encoder_BInputDigFltr_TimebaseSrc = 0x30e9

const DAQmx_CI_Velocity_Encoder_BInputDigFltr_TimebaseRate = 0x30ea

const DAQmx_CI_Velocity_MeasTime = 0x30eb

const DAQmx_CI_Velocity_Div = 0x30ec

const DAQmx_CI_TwoEdgeSep_Units = 0x18ac

const DAQmx_CI_TwoEdgeSep_FirstTerm = 0x18ad

const DAQmx_CI_TwoEdgeSep_FirstTermCfg = 0x30ab

const DAQmx_CI_TwoEdgeSep_FirstLogicLvlBehavior = 0x30ac

const DAQmx_CI_TwoEdgeSep_First_DigFltr_Enable = 0x220f

const DAQmx_CI_TwoEdgeSep_First_DigFltr_MinPulseWidth = 0x2210

const DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseSrc = 0x2211

const DAQmx_CI_TwoEdgeSep_First_DigFltr_TimebaseRate = 0x2212

const DAQmx_CI_TwoEdgeSep_First_DigSync_Enable = 0x2213

const DAQmx_CI_TwoEdgeSep_FirstEdge = 0x0833

const DAQmx_CI_TwoEdgeSep_SecondTerm = 0x18ae

const DAQmx_CI_TwoEdgeSep_SecondTermCfg = 0x30ad

const DAQmx_CI_TwoEdgeSep_SecondLogicLvlBehavior = 0x30ae

const DAQmx_CI_TwoEdgeSep_Second_DigFltr_Enable = 0x2214

const DAQmx_CI_TwoEdgeSep_Second_DigFltr_MinPulseWidth = 0x2215

const DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseSrc = 0x2216

const DAQmx_CI_TwoEdgeSep_Second_DigFltr_TimebaseRate = 0x2217

const DAQmx_CI_TwoEdgeSep_Second_DigSync_Enable = 0x2218

const DAQmx_CI_TwoEdgeSep_SecondEdge = 0x0834

const DAQmx_CI_SemiPeriod_Units = 0x18af

const DAQmx_CI_SemiPeriod_Term = 0x18b0

const DAQmx_CI_SemiPeriod_TermCfg = 0x30af

const DAQmx_CI_SemiPeriod_LogicLvlBehavior = 0x30b0

const DAQmx_CI_SemiPeriod_DigFltr_Enable = 0x2219

const DAQmx_CI_SemiPeriod_DigFltr_MinPulseWidth = 0x221a

const DAQmx_CI_SemiPeriod_DigFltr_TimebaseSrc = 0x221b

const DAQmx_CI_SemiPeriod_DigFltr_TimebaseRate = 0x221c

const DAQmx_CI_SemiPeriod_DigSync_Enable = 0x221d

const DAQmx_CI_SemiPeriod_StartingEdge = 0x22fe

const DAQmx_CI_Pulse_Freq_Units = 0x2f0b

const DAQmx_CI_Pulse_Freq_Term = 0x2f04

const DAQmx_CI_Pulse_Freq_TermCfg = 0x30b1

const DAQmx_CI_Pulse_Freq_LogicLvlBehavior = 0x30b2

const DAQmx_CI_Pulse_Freq_DigFltr_Enable = 0x2f06

const DAQmx_CI_Pulse_Freq_DigFltr_MinPulseWidth = 0x2f07

const DAQmx_CI_Pulse_Freq_DigFltr_TimebaseSrc = 0x2f08

const DAQmx_CI_Pulse_Freq_DigFltr_TimebaseRate = 0x2f09

const DAQmx_CI_Pulse_Freq_DigSync_Enable = 0x2f0a

const DAQmx_CI_Pulse_Freq_Start_Edge = 0x2f05

const DAQmx_CI_Pulse_Time_Units = 0x2f13

const DAQmx_CI_Pulse_Time_Term = 0x2f0c

const DAQmx_CI_Pulse_Time_TermCfg = 0x30b3

const DAQmx_CI_Pulse_Time_LogicLvlBehavior = 0x30b4

const DAQmx_CI_Pulse_Time_DigFltr_Enable = 0x2f0e

const DAQmx_CI_Pulse_Time_DigFltr_MinPulseWidth = 0x2f0f

const DAQmx_CI_Pulse_Time_DigFltr_TimebaseSrc = 0x2f10

const DAQmx_CI_Pulse_Time_DigFltr_TimebaseRate = 0x2f11

const DAQmx_CI_Pulse_Time_DigSync_Enable = 0x2f12

const DAQmx_CI_Pulse_Time_StartEdge = 0x2f0d

const DAQmx_CI_Pulse_Ticks_Term = 0x2f14

const DAQmx_CI_Pulse_Ticks_TermCfg = 0x30b5

const DAQmx_CI_Pulse_Ticks_LogicLvlBehavior = 0x30b6

const DAQmx_CI_Pulse_Ticks_DigFltr_Enable = 0x2f16

const DAQmx_CI_Pulse_Ticks_DigFltr_MinPulseWidth = 0x2f17

const DAQmx_CI_Pulse_Ticks_DigFltr_TimebaseSrc = 0x2f18

const DAQmx_CI_Pulse_Ticks_DigFltr_TimebaseRate = 0x2f19

const DAQmx_CI_Pulse_Ticks_DigSync_Enable = 0x2f1a

const DAQmx_CI_Pulse_Ticks_StartEdge = 0x2f15

const DAQmx_CI_CtrTimebaseSrc = 0x0143

const DAQmx_CI_CtrTimebaseRate = 0x18b2

const DAQmx_CI_CtrTimebaseActiveEdge = 0x0142

const DAQmx_CI_CtrTimebase_DigFltr_Enable = 0x2271

const DAQmx_CI_CtrTimebase_DigFltr_MinPulseWidth = 0x2272

const DAQmx_CI_CtrTimebase_DigFltr_TimebaseSrc = 0x2273

const DAQmx_CI_CtrTimebase_DigFltr_TimebaseRate = 0x2274

const DAQmx_CI_CtrTimebase_DigSync_Enable = 0x2275

const DAQmx_CI_ThreshVoltage = 0x30b7

const DAQmx_CI_Filter_Enable = 0x31b7

const DAQmx_CI_Filter_Freq = 0x31b8

const DAQmx_CI_Filter_Response = 0x31b9

const DAQmx_CI_Filter_Order = 0x31ba

const DAQmx_CI_FilterDelay = 0x31bb

const DAQmx_CI_FilterDelayUnits = 0x31bc

const DAQmx_CI_Count = 0x0148

const DAQmx_CI_OutputState = 0x0149

const DAQmx_CI_TCReached = 0x0150

const DAQmx_CI_CtrTimebaseMasterTimebaseDiv = 0x18b3

const DAQmx_CI_SampClkOverrunBehavior = 0x3093

const DAQmx_CI_SampClkOverrunSentinelVal = 0x3094

const DAQmx_CI_DataXferMech = 0x0200

const DAQmx_CI_DataXferReqCond = 0x2efb

const DAQmx_CI_UsbXferReqSize = 0x2a92

const DAQmx_CI_UsbXferReqCount = 0x3004

const DAQmx_CI_MemMapEnable = 0x2ed2

const DAQmx_CI_NumPossiblyInvalidSamps = 0x193c

const DAQmx_CI_DupCountPrevent = 0x21ac

const DAQmx_CI_Prescaler = 0x2239

const DAQmx_CI_MaxMeasPeriod = 0x3095

const DAQmx_CO_OutputType = 0x18b5

const DAQmx_CO_Pulse_IdleState = 0x1170

const DAQmx_CO_Pulse_Term = 0x18e1

const DAQmx_CO_Pulse_Time_Units = 0x18d6

const DAQmx_CO_Pulse_HighTime = 0x18ba

const DAQmx_CO_Pulse_LowTime = 0x18bb

const DAQmx_CO_Pulse_Time_InitialDelay = 0x18bc

const DAQmx_CO_Pulse_DutyCyc = 0x1176

const DAQmx_CO_Pulse_Freq_Units = 0x18d5

const DAQmx_CO_Pulse_Freq = 0x1178

const DAQmx_CO_Pulse_Freq_InitialDelay = 0x0299

const DAQmx_CO_Pulse_HighTicks = 0x1169

const DAQmx_CO_Pulse_LowTicks = 0x1171

const DAQmx_CO_Pulse_Ticks_InitialDelay = 0x0298

const DAQmx_CO_CtrTimebaseSrc = 0x0339

const DAQmx_CO_CtrTimebaseRate = 0x18c2

const DAQmx_CO_CtrTimebaseActiveEdge = 0x0341

const DAQmx_CO_CtrTimebase_DigFltr_Enable = 0x2276

const DAQmx_CO_CtrTimebase_DigFltr_MinPulseWidth = 0x2277

const DAQmx_CO_CtrTimebase_DigFltr_TimebaseSrc = 0x2278

const DAQmx_CO_CtrTimebase_DigFltr_TimebaseRate = 0x2279

const DAQmx_CO_CtrTimebase_DigSync_Enable = 0x227a

const DAQmx_CO_Count = 0x0293

const DAQmx_CO_OutputState = 0x0294

const DAQmx_CO_AutoIncrCnt = 0x0295

const DAQmx_CO_CtrTimebaseMasterTimebaseDiv = 0x18c3

const DAQmx_CO_PulseDone = 0x190e

const DAQmx_CO_EnableInitialDelayOnRetrigger = 0x2ec9

const DAQmx_CO_ConstrainedGenMode = 0x29f2

const DAQmx_CO_UseOnlyOnBrdMem = 0x2ecb

const DAQmx_CO_DataXferMech = 0x2ecc

const DAQmx_CO_DataXferReqCond = 0x2ecd

const DAQmx_CO_UsbXferReqSize = 0x2a93

const DAQmx_CO_UsbXferReqCount = 0x3005

const DAQmx_CO_MemMapEnable = 0x2ed3

const DAQmx_CO_Prescaler = 0x226d

const DAQmx_CO_RdyForNewVal = 0x22ff

const DAQmx_Pwr_Voltage_Setpoint = 0x31d4

const DAQmx_Pwr_Voltage_DevScalingCoeff = 0x31d9

const DAQmx_Pwr_Current_Setpoint = 0x31d5

const DAQmx_Pwr_Current_DevScalingCoeff = 0x31da

const DAQmx_Pwr_OutputEnable = 0x31d6

const DAQmx_Pwr_OutputState = 0x31d7

const DAQmx_Pwr_IdleOutputBehavior = 0x31d8

const DAQmx_Pwr_RemoteSense = 0x31db

const DAQmx_ChanType = 0x187f

const DAQmx_PhysicalChanName = 0x18f5

const DAQmx_ChanDescr = 0x1926

const DAQmx_ChanIsGlobal = 0x2304

const DAQmx_Chan_SyncUnlockBehavior = 0x313c

const DAQmx_Dev_IsSimulated = 0x22ca

const DAQmx_Dev_ProductCategory = 0x29a9

const DAQmx_Dev_ProductType = 0x0631

const DAQmx_Dev_ProductNum = 0x231d

const DAQmx_Dev_SerialNum = 0x0632

const DAQmx_Dev_Accessory_ProductTypes = 0x2f6d

const DAQmx_Dev_Accessory_ProductNums = 0x2f6e

const DAQmx_Dev_Accessory_SerialNums = 0x2f6f

const DAQmx_Carrier_SerialNum = 0x2a8a

const DAQmx_FieldDAQ_DevName = 0x3171

const DAQmx_FieldDAQ_BankDevNames = 0x3178

const DAQmx_Dev_Chassis_ModuleDevNames = 0x29b6

const DAQmx_Dev_AnlgTrigSupported = 0x2984

const DAQmx_Dev_DigTrigSupported = 0x2985

const DAQmx_Dev_TimeTrigSupported = 0x301f

const DAQmx_Dev_AI_PhysicalChans = 0x231e

const DAQmx_Dev_AI_SupportedMeasTypes = 0x2fd2

const DAQmx_Dev_AI_MaxSingleChanRate = 0x298c

const DAQmx_Dev_AI_MaxMultiChanRate = 0x298d

const DAQmx_Dev_AI_MinRate = 0x298e

const DAQmx_Dev_AI_SimultaneousSamplingSupported = 0x298f

const DAQmx_Dev_AI_NumSampTimingEngines = 0x3163

const DAQmx_Dev_AI_SampModes = 0x2fdc

const DAQmx_Dev_AI_NumSyncPulseSrcs = 0x3164

const DAQmx_Dev_AI_TrigUsage = 0x2986

const DAQmx_Dev_AI_VoltageRngs = 0x2990

const DAQmx_Dev_AI_VoltageIntExcitDiscreteVals = 0x29c9

const DAQmx_Dev_AI_VoltageIntExcitRangeVals = 0x29ca

const DAQmx_Dev_AI_ChargeRngs = 0x3111

const DAQmx_Dev_AI_CurrentRngs = 0x2991

const DAQmx_Dev_AI_CurrentIntExcitDiscreteVals = 0x29cb

const DAQmx_Dev_AI_BridgeRngs = 0x2fd0

const DAQmx_Dev_AI_ResistanceRngs = 0x2a15

const DAQmx_Dev_AI_FreqRngs = 0x2992

const DAQmx_Dev_AI_Gains = 0x2993

const DAQmx_Dev_AI_Couplings = 0x2994

const DAQmx_Dev_AI_LowpassCutoffFreqDiscreteVals = 0x2995

const DAQmx_Dev_AI_LowpassCutoffFreqRangeVals = 0x29cf

const DAQmx_AI_DigFltr_Types = 0x3107

const DAQmx_Dev_AI_DigFltr_LowpassCutoffFreqDiscreteVals = 0x30c8

const DAQmx_Dev_AI_DigFltr_LowpassCutoffFreqRangeVals = 0x30c9

const DAQmx_Dev_AO_PhysicalChans = 0x231f

const DAQmx_Dev_AO_SupportedOutputTypes = 0x2fd3

const DAQmx_Dev_AO_MaxRate = 0x2997

const DAQmx_Dev_AO_MinRate = 0x2998

const DAQmx_Dev_AO_SampClkSupported = 0x2996

const DAQmx_Dev_AO_NumSampTimingEngines = 0x3165

const DAQmx_Dev_AO_SampModes = 0x2fdd

const DAQmx_Dev_AO_NumSyncPulseSrcs = 0x3166

const DAQmx_Dev_AO_TrigUsage = 0x2987

const DAQmx_Dev_AO_VoltageRngs = 0x299b

const DAQmx_Dev_AO_CurrentRngs = 0x299c

const DAQmx_Dev_AO_Gains = 0x299d

const DAQmx_Dev_DI_Lines = 0x2320

const DAQmx_Dev_DI_Ports = 0x2321

const DAQmx_Dev_DI_MaxRate = 0x2999

const DAQmx_Dev_DI_NumSampTimingEngines = 0x3167

const DAQmx_Dev_DI_TrigUsage = 0x2988

const DAQmx_Dev_DO_Lines = 0x2322

const DAQmx_Dev_DO_Ports = 0x2323

const DAQmx_Dev_DO_MaxRate = 0x299a

const DAQmx_Dev_DO_NumSampTimingEngines = 0x3168

const DAQmx_Dev_DO_TrigUsage = 0x2989

const DAQmx_Dev_CI_PhysicalChans = 0x2324

const DAQmx_Dev_CI_SupportedMeasTypes = 0x2fd4

const DAQmx_Dev_CI_TrigUsage = 0x298a

const DAQmx_Dev_CI_SampClkSupported = 0x299e

const DAQmx_Dev_CI_SampModes = 0x2fde

const DAQmx_Dev_CI_MaxSize = 0x299f

const DAQmx_Dev_CI_MaxTimebase = 0x29a0

const DAQmx_Dev_CO_PhysicalChans = 0x2325

const DAQmx_Dev_CO_SupportedOutputTypes = 0x2fd5

const DAQmx_Dev_CO_SampClkSupported = 0x2f5b

const DAQmx_Dev_CO_SampModes = 0x2fdf

const DAQmx_Dev_CO_TrigUsage = 0x298b

const DAQmx_Dev_CO_MaxSize = 0x29a1

const DAQmx_Dev_CO_MaxTimebase = 0x29a2

const DAQmx_Dev_TEDS_HWTEDSSupported = 0x2fd6

const DAQmx_Dev_NumDMAChans = 0x233c

const DAQmx_Dev_BusType = 0x2326

const DAQmx_Dev_PCI_BusNum = 0x2327

const DAQmx_Dev_PCI_DevNum = 0x2328

const DAQmx_Dev_PXI_ChassisNum = 0x2329

const DAQmx_Dev_PXI_SlotNum = 0x232a

const DAQmx_Dev_CompactDAQ_ChassisDevName = 0x29b7

const DAQmx_Dev_CompactDAQ_SlotNum = 0x29b8

const DAQmx_Dev_CompactRIO_ChassisDevName = 0x3161

const DAQmx_Dev_CompactRIO_SlotNum = 0x3162

const DAQmx_Dev_TCPIP_Hostname = 0x2a8b

const DAQmx_Dev_TCPIP_EthernetIP = 0x2a8c

const DAQmx_Dev_TCPIP_WirelessIP = 0x2a8d

const DAQmx_Dev_Terminals = 0x2a40

const DAQmx_Dev_NumTimeTrigs = 0x3141

const DAQmx_Dev_NumTimestampEngines = 0x3142

const DAQmx_Exported_AIConvClk_OutputTerm = 0x1687

const DAQmx_Exported_AIConvClk_Pulse_Polarity = 0x1688

const DAQmx_Exported_10MHzRefClk_OutputTerm = 0x226e

const DAQmx_Exported_20MHzTimebase_OutputTerm = 0x1657

const DAQmx_Exported_SampClk_OutputBehavior = 0x186b

const DAQmx_Exported_SampClk_OutputTerm = 0x1663

const DAQmx_Exported_SampClk_DelayOffset = 0x21c4

const DAQmx_Exported_SampClk_Pulse_Polarity = 0x1664

const DAQmx_Exported_SampClkTimebase_OutputTerm = 0x18f9

const DAQmx_Exported_DividedSampClkTimebase_OutputTerm = 0x21a1

const DAQmx_Exported_AdvTrig_OutputTerm = 0x1645

const DAQmx_Exported_AdvTrig_Pulse_Polarity = 0x1646

const DAQmx_Exported_AdvTrig_Pulse_WidthUnits = 0x1647

const DAQmx_Exported_AdvTrig_Pulse_Width = 0x1648

const DAQmx_Exported_PauseTrig_OutputTerm = 0x1615

const DAQmx_Exported_PauseTrig_Lvl_ActiveLvl = 0x1616

const DAQmx_Exported_RefTrig_OutputTerm = 0x0590

const DAQmx_Exported_RefTrig_Pulse_Polarity = 0x0591

const DAQmx_Exported_StartTrig_OutputTerm = 0x0584

const DAQmx_Exported_StartTrig_Pulse_Polarity = 0x0585

const DAQmx_Exported_AdvCmpltEvent_OutputTerm = 0x1651

const DAQmx_Exported_AdvCmpltEvent_Delay = 0x1757

const DAQmx_Exported_AdvCmpltEvent_Pulse_Polarity = 0x1652

const DAQmx_Exported_AdvCmpltEvent_Pulse_Width = 0x1654

const DAQmx_Exported_AIHoldCmpltEvent_OutputTerm = 0x18ed

const DAQmx_Exported_AIHoldCmpltEvent_PulsePolarity = 0x18ee

const DAQmx_Exported_ChangeDetectEvent_OutputTerm = 0x2197

const DAQmx_Exported_ChangeDetectEvent_Pulse_Polarity = 0x2303

const DAQmx_Exported_CtrOutEvent_OutputTerm = 0x1717

const DAQmx_Exported_CtrOutEvent_OutputBehavior = 0x174f

const DAQmx_Exported_CtrOutEvent_Pulse_Polarity = 0x1718

const DAQmx_Exported_CtrOutEvent_Toggle_IdleState = 0x186a

const DAQmx_Exported_HshkEvent_OutputTerm = 0x22ba

const DAQmx_Exported_HshkEvent_OutputBehavior = 0x22bb

const DAQmx_Exported_HshkEvent_Delay = 0x22bc

const DAQmx_Exported_HshkEvent_Interlocked_AssertedLvl = 0x22bd

const DAQmx_Exported_HshkEvent_Interlocked_AssertOnStart = 0x22be

const DAQmx_Exported_HshkEvent_Interlocked_DeassertDelay = 0x22bf

const DAQmx_Exported_HshkEvent_Pulse_Polarity = 0x22c0

const DAQmx_Exported_HshkEvent_Pulse_Width = 0x22c1

const DAQmx_Exported_RdyForXferEvent_OutputTerm = 0x22b5

const DAQmx_Exported_RdyForXferEvent_Lvl_ActiveLvl = 0x22b6

const DAQmx_Exported_RdyForXferEvent_DeassertCond = 0x2963

const DAQmx_Exported_RdyForXferEvent_DeassertCondCustomThreshold = 0x2964

const DAQmx_Exported_DataActiveEvent_OutputTerm = 0x1633

const DAQmx_Exported_DataActiveEvent_Lvl_ActiveLvl = 0x1634

const DAQmx_Exported_RdyForStartEvent_OutputTerm = 0x1609

const DAQmx_Exported_RdyForStartEvent_Lvl_ActiveLvl = 0x1751

const DAQmx_Exported_SyncPulseEvent_OutputTerm = 0x223c

const DAQmx_Exported_WatchdogExpiredEvent_OutputTerm = 0x21aa

const DAQmx_PersistedChan_Author = 0x22d0

const DAQmx_PersistedChan_AllowInteractiveEditing = 0x22d1

const DAQmx_PersistedChan_AllowInteractiveDeletion = 0x22d2

const DAQmx_PersistedScale_Author = 0x22d4

const DAQmx_PersistedScale_AllowInteractiveEditing = 0x22d5

const DAQmx_PersistedScale_AllowInteractiveDeletion = 0x22d6

const DAQmx_PersistedTask_Author = 0x22cc

const DAQmx_PersistedTask_AllowInteractiveEditing = 0x22cd

const DAQmx_PersistedTask_AllowInteractiveDeletion = 0x22ce

const DAQmx_PhysicalChan_AI_SupportedMeasTypes = 0x2fd7

const DAQmx_PhysicalChan_AI_TermCfgs = 0x2342

const DAQmx_PhysicalChan_AI_InputSrcs = 0x2fd8

const DAQmx_PhysicalChan_AI_SensorPower_Types = 0x3179

const DAQmx_PhysicalChan_AI_SensorPower_VoltageRangeVals = 0x317a

const DAQmx_PhysicalChan_AI_PowerControl_Voltage = 0x316c

const DAQmx_PhysicalChan_AI_PowerControl_Enable = 0x316d

const DAQmx_PhysicalChan_AI_PowerControl_Type = 0x316e

const DAQmx_PhysicalChan_AI_SensorPower_OpenChan = 0x317c

const DAQmx_PhysicalChan_AI_SensorPower_Overcurrent = 0x317d

const DAQmx_PhysicalChan_AO_SupportedOutputTypes = 0x2fd9

const DAQmx_PhysicalChan_AO_SupportedPowerUpOutputTypes = 0x304e

const DAQmx_PhysicalChan_AO_TermCfgs = 0x29a3

const DAQmx_PhysicalChan_AO_ManualControlEnable = 0x2a1e

const DAQmx_PhysicalChan_AO_ManualControl_ShortDetected = 0x2ec3

const DAQmx_PhysicalChan_AO_ManualControlAmplitude = 0x2a1f

const DAQmx_PhysicalChan_AO_ManualControlFreq = 0x2a20

const DAQmx_AO_PowerAmp_ChannelEnable = 0x3062

const DAQmx_AO_PowerAmp_ScalingCoeff = 0x3063

const DAQmx_AO_PowerAmp_Overcurrent = 0x3064

const DAQmx_AO_PowerAmp_Gain = 0x3065

const DAQmx_AO_PowerAmp_Offset = 0x3066

const DAQmx_PhysicalChan_DI_PortWidth = 0x29a4

const DAQmx_PhysicalChan_DI_SampClkSupported = 0x29a5

const DAQmx_PhysicalChan_DI_SampModes = 0x2fe0

const DAQmx_PhysicalChan_DI_ChangeDetectSupported = 0x29a6

const DAQmx_PhysicalChan_DO_PortWidth = 0x29a7

const DAQmx_PhysicalChan_DO_SampClkSupported = 0x29a8

const DAQmx_PhysicalChan_DO_SampModes = 0x2fe1

const DAQmx_PhysicalChan_CI_SupportedMeasTypes = 0x2fda

const DAQmx_PhysicalChan_CO_SupportedOutputTypes = 0x2fdb

const DAQmx_PhysicalChan_TEDS_MfgID = 0x21da

const DAQmx_PhysicalChan_TEDS_ModelNum = 0x21db

const DAQmx_PhysicalChan_TEDS_SerialNum = 0x21dc

const DAQmx_PhysicalChan_TEDS_VersionNum = 0x21dd

const DAQmx_PhysicalChan_TEDS_VersionLetter = 0x21de

const DAQmx_PhysicalChan_TEDS_BitStream = 0x21df

const DAQmx_PhysicalChan_TEDS_TemplateIDs = 0x228f

const DAQmx_Read_RelativeTo = 0x190a

const DAQmx_Read_Offset = 0x190b

const DAQmx_Read_ChannelsToRead = 0x1823

const DAQmx_Read_ReadAllAvailSamp = 0x1215

const DAQmx_Read_AutoStart = 0x1826

const DAQmx_Read_OverWrite = 0x1211

const DAQmx_Logging_FilePath = 0x2ec4

const DAQmx_Logging_Mode = 0x2ec5

const DAQmx_Logging_TDMS_GroupName = 0x2ec6

const DAQmx_Logging_TDMS_Operation = 0x2ec7

const DAQmx_Logging_Pause = 0x2fe3

const DAQmx_Logging_SampsPerFile = 0x2fe4

const DAQmx_Logging_FileWriteSize = 0x2fc3

const DAQmx_Logging_FilePreallocationSize = 0x2fc6

const DAQmx_Read_CurrReadPos = 0x1221

const DAQmx_Read_AvailSampPerChan = 0x1223

const DAQmx_Read_TotalSampPerChanAcquired = 0x192a

const DAQmx_Read_CommonModeRangeErrorChansExist = 0x2a98

const DAQmx_Read_CommonModeRangeErrorChans = 0x2a99

const DAQmx_Read_ExcitFaultChansExist = 0x3088

const DAQmx_Read_ExcitFaultChans = 0x3089

const DAQmx_Read_OvercurrentChansExist = 0x29e6

const DAQmx_Read_OvercurrentChans = 0x29e7

const DAQmx_Read_OvertemperatureChansExist = 0x3081

const DAQmx_Read_OvertemperatureChans = 0x3082

const DAQmx_Read_OpenChansExist = 0x3100

const DAQmx_Read_OpenChans = 0x3101

const DAQmx_Read_OpenChansDetails = 0x3102

const DAQmx_Read_OpenCurrentLoopChansExist = 0x2a09

const DAQmx_Read_OpenCurrentLoopChans = 0x2a0a

const DAQmx_Read_OpenThrmcplChansExist = 0x2a96

const DAQmx_Read_OpenThrmcplChans = 0x2a97

const DAQmx_Read_OverloadedChansExist = 0x2174

const DAQmx_Read_OverloadedChans = 0x2175

const DAQmx_Read_InputLimitsFaultChansExist = 0x318f

const DAQmx_Read_InputLimitsFaultChans = 0x3190

const DAQmx_Read_PLL_UnlockedChansExist = 0x3118

const DAQmx_Read_PLL_UnlockedChans = 0x3119

const DAQmx_Read_PowerSupplyFaultChansExist = 0x3192

const DAQmx_Read_PowerSupplyFaultChans = 0x3193

const DAQmx_Read_Sync_UnlockedChansExist = 0x313d

const DAQmx_Read_Sync_UnlockedChans = 0x313e

const DAQmx_Read_AccessoryInsertionOrRemovalDetected = 0x2f70

const DAQmx_Read_DevsWithInsertedOrRemovedAccessories = 0x2f71

const DAQmx_RemoteSenseErrorChansExist = 0x31dd

const DAQmx_RemoteSenseErrorChans = 0x31de

const DAQmx_AuxPowerErrorChansExist = 0x31df

const DAQmx_AuxPowerErrorChans = 0x31e0

const DAQmx_ReverseVoltageErrorChansExist = 0x31e6

const DAQmx_ReverseVoltageErrorChans = 0x31e7

const DAQmx_Read_ChangeDetect_HasOverflowed = 0x2194

const DAQmx_Read_RawDataWidth = 0x217a

const DAQmx_Read_NumChans = 0x217b

const DAQmx_Read_DigitalLines_BytesPerChan = 0x217c

const DAQmx_Read_WaitMode = 0x2232

const DAQmx_Read_SleepTime = 0x22b0

const DAQmx_RealTime_ConvLateErrorsToWarnings = 0x22ee

const DAQmx_RealTime_NumOfWarmupIters = 0x22ed

const DAQmx_RealTime_WaitForNextSampClkWaitMode = 0x22ef

const DAQmx_RealTime_ReportMissedSamp = 0x2319

const DAQmx_RealTime_WriteRecoveryMode = 0x231a

const DAQmx_Scale_Descr = 0x1226

const DAQmx_Scale_ScaledUnits = 0x191b

const DAQmx_Scale_PreScaledUnits = 0x18f7

const DAQmx_Scale_Type = 0x1929

const DAQmx_Scale_Lin_Slope = 0x1227

const DAQmx_Scale_Lin_YIntercept = 0x1228

const DAQmx_Scale_Map_ScaledMax = 0x1229

const DAQmx_Scale_Map_PreScaledMax = 0x1231

const DAQmx_Scale_Map_ScaledMin = 0x1230

const DAQmx_Scale_Map_PreScaledMin = 0x1232

const DAQmx_Scale_Poly_ForwardCoeff = 0x1234

const DAQmx_Scale_Poly_ReverseCoeff = 0x1235

const DAQmx_Scale_Table_ScaledVals = 0x1236

const DAQmx_Scale_Table_PreScaledVals = 0x1237

const DAQmx_SwitchChan_Usage = 0x18e4

const DAQmx_SwitchChan_AnlgBusSharingEnable = 0x2f9e

const DAQmx_SwitchChan_MaxACCarryCurrent = 0x0648

const DAQmx_SwitchChan_MaxACSwitchCurrent = 0x0646

const DAQmx_SwitchChan_MaxACCarryPwr = 0x0642

const DAQmx_SwitchChan_MaxACSwitchPwr = 0x0644

const DAQmx_SwitchChan_MaxDCCarryCurrent = 0x0647

const DAQmx_SwitchChan_MaxDCSwitchCurrent = 0x0645

const DAQmx_SwitchChan_MaxDCCarryPwr = 0x0643

const DAQmx_SwitchChan_MaxDCSwitchPwr = 0x0649

const DAQmx_SwitchChan_MaxACVoltage = 0x0651

const DAQmx_SwitchChan_MaxDCVoltage = 0x0650

const DAQmx_SwitchChan_WireMode = 0x18e5

const DAQmx_SwitchChan_Bandwidth = 0x0640

const DAQmx_SwitchChan_Impedance = 0x0641

const DAQmx_SwitchDev_SettlingTime = 0x1244

const DAQmx_SwitchDev_AutoConnAnlgBus = 0x17da

const DAQmx_SwitchDev_PwrDownLatchRelaysAfterSettling = 0x22db

const DAQmx_SwitchDev_Settled = 0x1243

const DAQmx_SwitchDev_RelayList = 0x17dc

const DAQmx_SwitchDev_NumRelays = 0x18e6

const DAQmx_SwitchDev_SwitchChanList = 0x18e7

const DAQmx_SwitchDev_NumSwitchChans = 0x18e8

const DAQmx_SwitchDev_NumRows = 0x18e9

const DAQmx_SwitchDev_NumColumns = 0x18ea

const DAQmx_SwitchDev_Topology = 0x193d

const DAQmx_SwitchDev_Temperature = 0x301a

const DAQmx_SwitchScan_BreakMode = 0x1247

const DAQmx_SwitchScan_RepeatMode = 0x1248

const DAQmx_SwitchScan_WaitingForAdv = 0x17d9

const DAQmx_Sys_GlobalChans = 0x1265

const DAQmx_Sys_Scales = 0x1266

const DAQmx_Sys_Tasks = 0x1267

const DAQmx_Sys_DevNames = 0x193b

const DAQmx_Sys_NIDAQMajorVersion = 0x1272

const DAQmx_Sys_NIDAQMinorVersion = 0x1923

const DAQmx_Sys_NIDAQUpdateVersion = 0x2f22

const DAQmx_Task_Name = 0x1276

const DAQmx_Task_Channels = 0x1273

const DAQmx_Task_NumChans = 0x2181

const DAQmx_Task_Devices = 0x230e

const DAQmx_Task_NumDevices = 0x29ba

const DAQmx_Task_Complete = 0x1274

const DAQmx_SampQuant_SampMode = 0x1300

const DAQmx_SampQuant_SampPerChan = 0x1310

const DAQmx_SampTimingType = 0x1347

const DAQmx_SampClk_Rate = 0x1344

const DAQmx_SampClk_MaxRate = 0x22c8

const DAQmx_SampClk_Src = 0x1852

const DAQmx_SampClk_ActiveEdge = 0x1301

const DAQmx_SampClk_OverrunBehavior = 0x2efc

const DAQmx_SampClk_UnderflowBehavior = 0x2961

const DAQmx_SampClk_TimebaseDiv = 0x18eb

const DAQmx_SampClk_Term = 0x2f1b

const DAQmx_SampClk_Timebase_Rate = 0x1303

const DAQmx_SampClk_Timebase_Src = 0x1308

const DAQmx_SampClk_Timebase_ActiveEdge = 0x18ec

const DAQmx_SampClk_Timebase_MasterTimebaseDiv = 0x1305

const DAQmx_SampClkTimebase_Term = 0x2f1c

const DAQmx_SampClk_DigFltr_Enable = 0x221e

const DAQmx_SampClk_DigFltr_MinPulseWidth = 0x221f

const DAQmx_SampClk_DigFltr_TimebaseSrc = 0x2220

const DAQmx_SampClk_DigFltr_TimebaseRate = 0x2221

const DAQmx_SampClk_DigSync_Enable = 0x2222

const DAQmx_SampClk_WriteWfm_UseInitialWfmDT = 0x30fc

const DAQmx_Hshk_DelayAfterXfer = 0x22c2

const DAQmx_Hshk_StartCond = 0x22c3

const DAQmx_Hshk_SampleInputDataWhen = 0x22c4

const DAQmx_ChangeDetect_DI_RisingEdgePhysicalChans = 0x2195

const DAQmx_ChangeDetect_DI_FallingEdgePhysicalChans = 0x2196

const DAQmx_ChangeDetect_DI_Tristate = 0x2efa

const DAQmx_OnDemand_SimultaneousAOEnable = 0x21a0

const DAQmx_Implicit_UnderflowBehavior = 0x2efd

const DAQmx_AIConv_Rate = 0x1848

const DAQmx_AIConv_MaxRate = 0x22c9

const DAQmx_AIConv_Src = 0x1502

const DAQmx_AIConv_ActiveEdge = 0x1853

const DAQmx_AIConv_TimebaseDiv = 0x1335

const DAQmx_AIConv_Timebase_Src = 0x1339

const DAQmx_DelayFromSampClk_DelayUnits = 0x1304

const DAQmx_DelayFromSampClk_Delay = 0x1317

const DAQmx_AIConv_DigFltr_Enable = 0x2edc

const DAQmx_AIConv_DigFltr_MinPulseWidth = 0x2edd

const DAQmx_AIConv_DigFltr_TimebaseSrc = 0x2ede

const DAQmx_AIConv_DigFltr_TimebaseRate = 0x2edf

const DAQmx_AIConv_DigSync_Enable = 0x2ee0

const DAQmx_MasterTimebase_Rate = 0x1495

const DAQmx_MasterTimebase_Src = 0x1343

const DAQmx_RefClk_Rate = 0x1315

const DAQmx_RefClk_Src = 0x1316

const DAQmx_SyncPulse_Type = 0x3136

const DAQmx_SyncPulse_Src = 0x223d

const DAQmx_SyncPulse_Time_When = 0x3137

const DAQmx_SyncPulse_Time_Timescale = 0x3138

const DAQmx_SyncPulse_SyncTime = 0x223e

const DAQmx_SyncPulse_MinDelayToStart = 0x223f

const DAQmx_SyncPulse_ResetTime = 0x2f7c

const DAQmx_SyncPulse_ResetDelay = 0x2f7d

const DAQmx_SyncPulse_Term = 0x2f85

const DAQmx_SyncClk_Interval = 0x2f7e

const DAQmx_SampTimingEngine = 0x2a26

const DAQmx_FirstSampTimestamp_Enable = 0x3139

const DAQmx_FirstSampTimestamp_Timescale = 0x313b

const DAQmx_FirstSampTimestamp_Val = 0x313a

const DAQmx_FirstSampClk_When = 0x3182

const DAQmx_FirstSampClk_Timescale = 0x3183

const DAQmx_FirstSampClk_Offset = 0x31aa

const DAQmx_StartTrig_Type = 0x1393

const DAQmx_StartTrig_Term = 0x2f1e

const DAQmx_DigEdge_StartTrig_Src = 0x1407

const DAQmx_DigEdge_StartTrig_Edge = 0x1404

const DAQmx_DigEdge_StartTrig_DigFltr_Enable = 0x2223

const DAQmx_DigEdge_StartTrig_DigFltr_MinPulseWidth = 0x2224

const DAQmx_DigEdge_StartTrig_DigFltr_TimebaseSrc = 0x2225

const DAQmx_DigEdge_StartTrig_DigFltr_TimebaseRate = 0x2226

const DAQmx_DigEdge_StartTrig_DigSync_Enable = 0x2227

const DAQmx_DigPattern_StartTrig_Src = 0x1410

const DAQmx_DigPattern_StartTrig_Pattern = 0x2186

const DAQmx_DigPattern_StartTrig_When = 0x1411

const DAQmx_AnlgEdge_StartTrig_Src = 0x1398

const DAQmx_AnlgEdge_StartTrig_Slope = 0x1397

const DAQmx_AnlgEdge_StartTrig_Lvl = 0x1396

const DAQmx_AnlgEdge_StartTrig_Hyst = 0x1395

const DAQmx_AnlgEdge_StartTrig_Coupling = 0x2233

const DAQmx_AnlgEdge_StartTrig_DigFltr_Enable = 0x2ee1

const DAQmx_AnlgEdge_StartTrig_DigFltr_MinPulseWidth = 0x2ee2

const DAQmx_AnlgEdge_StartTrig_DigFltr_TimebaseSrc = 0x2ee3

const DAQmx_AnlgEdge_StartTrig_DigFltr_TimebaseRate = 0x2ee4

const DAQmx_AnlgEdge_StartTrig_DigSync_Enable = 0x2ee5

const DAQmx_AnlgMultiEdge_StartTrig_Srcs = 0x3121

const DAQmx_AnlgMultiEdge_StartTrig_Slopes = 0x3122

const DAQmx_AnlgMultiEdge_StartTrig_Lvls = 0x3123

const DAQmx_AnlgMultiEdge_StartTrig_Hysts = 0x3124

const DAQmx_AnlgMultiEdge_StartTrig_Couplings = 0x3125

const DAQmx_AnlgWin_StartTrig_Src = 0x1400

const DAQmx_AnlgWin_StartTrig_When = 0x1401

const DAQmx_AnlgWin_StartTrig_Top = 0x1403

const DAQmx_AnlgWin_StartTrig_Btm = 0x1402

const DAQmx_AnlgWin_StartTrig_Coupling = 0x2234

const DAQmx_AnlgWin_StartTrig_DigFltr_Enable = 0x2eff

const DAQmx_AnlgWin_StartTrig_DigFltr_MinPulseWidth = 0x2f00

const DAQmx_AnlgWin_StartTrig_DigFltr_TimebaseSrc = 0x2f01

const DAQmx_AnlgWin_StartTrig_DigFltr_TimebaseRate = 0x2f02

const DAQmx_AnlgWin_StartTrig_DigSync_Enable = 0x2f03

const DAQmx_StartTrig_TrigWhen = 0x304d

const DAQmx_StartTrig_Timescale = 0x3036

const DAQmx_StartTrig_TimestampEnable = 0x314a

const DAQmx_StartTrig_TimestampTimescale = 0x312d

const DAQmx_StartTrig_TimestampVal = 0x314b

const DAQmx_StartTrig_Delay = 0x1856

const DAQmx_StartTrig_DelayUnits = 0x18c8

const DAQmx_StartTrig_Retriggerable = 0x190f

const DAQmx_StartTrig_TrigWin = 0x311a

const DAQmx_StartTrig_RetriggerWin = 0x311b

const DAQmx_StartTrig_MaxNumTrigsToDetect = 0x311c

const DAQmx_RefTrig_Type = 0x1419

const DAQmx_RefTrig_PretrigSamples = 0x1445

const DAQmx_RefTrig_Term = 0x2f1f

const DAQmx_DigEdge_RefTrig_Src = 0x1434

const DAQmx_DigEdge_RefTrig_Edge = 0x1430

const DAQmx_DigEdge_RefTrig_DigFltr_Enable = 0x2ed7

const DAQmx_DigEdge_RefTrig_DigFltr_MinPulseWidth = 0x2ed8

const DAQmx_DigEdge_RefTrig_DigFltr_TimebaseSrc = 0x2ed9

const DAQmx_DigEdge_RefTrig_DigFltr_TimebaseRate = 0x2eda

const DAQmx_DigEdge_RefTrig_DigSync_Enable = 0x2edb

const DAQmx_DigPattern_RefTrig_Src = 0x1437

const DAQmx_DigPattern_RefTrig_Pattern = 0x2187

const DAQmx_DigPattern_RefTrig_When = 0x1438

const DAQmx_AnlgEdge_RefTrig_Src = 0x1424

const DAQmx_AnlgEdge_RefTrig_Slope = 0x1423

const DAQmx_AnlgEdge_RefTrig_Lvl = 0x1422

const DAQmx_AnlgEdge_RefTrig_Hyst = 0x1421

const DAQmx_AnlgEdge_RefTrig_Coupling = 0x2235

const DAQmx_AnlgEdge_RefTrig_DigFltr_Enable = 0x2ee6

const DAQmx_AnlgEdge_RefTrig_DigFltr_MinPulseWidth = 0x2ee7

const DAQmx_AnlgEdge_RefTrig_DigFltr_TimebaseSrc = 0x2ee8

const DAQmx_AnlgEdge_RefTrig_DigFltr_TimebaseRate = 0x2ee9

const DAQmx_AnlgEdge_RefTrig_DigSync_Enable = 0x2eea

const DAQmx_AnlgMultiEdge_RefTrig_Srcs = 0x3126

const DAQmx_AnlgMultiEdge_RefTrig_Slopes = 0x3127

const DAQmx_AnlgMultiEdge_RefTrig_Lvls = 0x3128

const DAQmx_AnlgMultiEdge_RefTrig_Hysts = 0x3129

const DAQmx_AnlgMultiEdge_RefTrig_Couplings = 0x312a

const DAQmx_AnlgWin_RefTrig_Src = 0x1426

const DAQmx_AnlgWin_RefTrig_When = 0x1427

const DAQmx_AnlgWin_RefTrig_Top = 0x1429

const DAQmx_AnlgWin_RefTrig_Btm = 0x1428

const DAQmx_AnlgWin_RefTrig_Coupling = 0x1857

const DAQmx_AnlgWin_RefTrig_DigFltr_Enable = 0x2eeb

const DAQmx_AnlgWin_RefTrig_DigFltr_MinPulseWidth = 0x2eec

const DAQmx_AnlgWin_RefTrig_DigFltr_TimebaseSrc = 0x2eed

const DAQmx_AnlgWin_RefTrig_DigFltr_TimebaseRate = 0x2eee

const DAQmx_AnlgWin_RefTrig_DigSync_Enable = 0x2eef

const DAQmx_RefTrig_AutoTrigEnable = 0x2ec1

const DAQmx_RefTrig_AutoTriggered = 0x2ec2

const DAQmx_RefTrig_TimestampEnable = 0x312e

const DAQmx_RefTrig_TimestampTimescale = 0x3130

const DAQmx_RefTrig_TimestampVal = 0x312f

const DAQmx_RefTrig_Delay = 0x1483

const DAQmx_RefTrig_Retriggerable = 0x311d

const DAQmx_RefTrig_TrigWin = 0x311e

const DAQmx_RefTrig_RetriggerWin = 0x311f

const DAQmx_RefTrig_MaxNumTrigsToDetect = 0x3120

const DAQmx_AdvTrig_Type = 0x1365

const DAQmx_DigEdge_AdvTrig_Src = 0x1362

const DAQmx_DigEdge_AdvTrig_Edge = 0x1360

const DAQmx_DigEdge_AdvTrig_DigFltr_Enable = 0x2238

const DAQmx_HshkTrig_Type = 0x22b7

const DAQmx_Interlocked_HshkTrig_Src = 0x22b8

const DAQmx_Interlocked_HshkTrig_AssertedLvl = 0x22b9

const DAQmx_PauseTrig_Type = 0x1366

const DAQmx_PauseTrig_Term = 0x2f20

const DAQmx_AnlgLvl_PauseTrig_Src = 0x1370

const DAQmx_AnlgLvl_PauseTrig_When = 0x1371

const DAQmx_AnlgLvl_PauseTrig_Lvl = 0x1369

const DAQmx_AnlgLvl_PauseTrig_Hyst = 0x1368

const DAQmx_AnlgLvl_PauseTrig_Coupling = 0x2236

const DAQmx_AnlgLvl_PauseTrig_DigFltr_Enable = 0x2ef0

const DAQmx_AnlgLvl_PauseTrig_DigFltr_MinPulseWidth = 0x2ef1

const DAQmx_AnlgLvl_PauseTrig_DigFltr_TimebaseSrc = 0x2ef2

const DAQmx_AnlgLvl_PauseTrig_DigFltr_TimebaseRate = 0x2ef3

const DAQmx_AnlgLvl_PauseTrig_DigSync_Enable = 0x2ef4

const DAQmx_AnlgWin_PauseTrig_Src = 0x1373

const DAQmx_AnlgWin_PauseTrig_When = 0x1374

const DAQmx_AnlgWin_PauseTrig_Top = 0x1376

const DAQmx_AnlgWin_PauseTrig_Btm = 0x1375

const DAQmx_AnlgWin_PauseTrig_Coupling = 0x2237

const DAQmx_AnlgWin_PauseTrig_DigFltr_Enable = 0x2ef5

const DAQmx_AnlgWin_PauseTrig_DigFltr_MinPulseWidth = 0x2ef6

const DAQmx_AnlgWin_PauseTrig_DigFltr_TimebaseSrc = 0x2ef7

const DAQmx_AnlgWin_PauseTrig_DigFltr_TimebaseRate = 0x2ef8

const DAQmx_AnlgWin_PauseTrig_DigSync_Enable = 0x2ef9

const DAQmx_DigLvl_PauseTrig_Src = 0x1379

const DAQmx_DigLvl_PauseTrig_When = 0x1380

const DAQmx_DigLvl_PauseTrig_DigFltr_Enable = 0x2228

const DAQmx_DigLvl_PauseTrig_DigFltr_MinPulseWidth = 0x2229

const DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseSrc = 0x222a

const DAQmx_DigLvl_PauseTrig_DigFltr_TimebaseRate = 0x222b

const DAQmx_DigLvl_PauseTrig_DigSync_Enable = 0x222c

const DAQmx_DigPattern_PauseTrig_Src = 0x216f

const DAQmx_DigPattern_PauseTrig_Pattern = 0x2188

const DAQmx_DigPattern_PauseTrig_When = 0x2170

const DAQmx_ArmStartTrig_Type = 0x1414

const DAQmx_ArmStart_Term = 0x2f7f

const DAQmx_DigEdge_ArmStartTrig_Src = 0x1417

const DAQmx_DigEdge_ArmStartTrig_Edge = 0x1415

const DAQmx_DigEdge_ArmStartTrig_DigFltr_Enable = 0x222d

const DAQmx_DigEdge_ArmStartTrig_DigFltr_MinPulseWidth = 0x222e

const DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseSrc = 0x222f

const DAQmx_DigEdge_ArmStartTrig_DigFltr_TimebaseRate = 0x2230

const DAQmx_DigEdge_ArmStartTrig_DigSync_Enable = 0x2231

const DAQmx_ArmStartTrig_TrigWhen = 0x3131

const DAQmx_ArmStartTrig_Timescale = 0x3132

const DAQmx_ArmStartTrig_TimestampEnable = 0x3133

const DAQmx_ArmStartTrig_TimestampTimescale = 0x3135

const DAQmx_ArmStartTrig_TimestampVal = 0x3134

const DAQmx_Trigger_SyncType = 0x2f80

const DAQmx_Watchdog_Timeout = 0x21a9

const DAQmx_WatchdogExpirTrig_Type = 0x21a3

const DAQmx_WatchdogExpirTrig_TrigOnNetworkConnLoss = 0x305d

const DAQmx_DigEdge_WatchdogExpirTrig_Src = 0x21a4

const DAQmx_DigEdge_WatchdogExpirTrig_Edge = 0x21a5

const DAQmx_Watchdog_DO_ExpirState = 0x21a7

const DAQmx_Watchdog_AO_OutputType = 0x305e

const DAQmx_Watchdog_AO_ExpirState = 0x305f

const DAQmx_Watchdog_CO_ExpirState = 0x3060

const DAQmx_Watchdog_HasExpired = 0x21a8

const DAQmx_Write_RelativeTo = 0x190c

const DAQmx_Write_Offset = 0x190d

const DAQmx_Write_RegenMode = 0x1453

const DAQmx_Write_CurrWritePos = 0x1458

const DAQmx_Write_OvercurrentChansExist = 0x29e8

const DAQmx_Write_OvercurrentChans = 0x29e9

const DAQmx_Write_OvertemperatureChansExist = 0x2a84

const DAQmx_Write_OvertemperatureChans = 0x3083

const DAQmx_Write_ExternalOvervoltageChansExist = 0x30bb

const DAQmx_Write_ExternalOvervoltageChans = 0x30bc

const DAQmx_Write_OverloadedChansExist = 0x3084

const DAQmx_Write_OverloadedChans = 0x3085

const DAQmx_Write_OpenCurrentLoopChansExist = 0x29ea

const DAQmx_Write_OpenCurrentLoopChans = 0x29eb

const DAQmx_Write_PowerSupplyFaultChansExist = 0x29ec

const DAQmx_Write_PowerSupplyFaultChans = 0x29ed

const DAQmx_Write_Sync_UnlockedChansExist = 0x313f

const DAQmx_Write_Sync_UnlockedChans = 0x3140

const DAQmx_Write_SpaceAvail = 0x1460

const DAQmx_Write_TotalSampPerChanGenerated = 0x192b

const DAQmx_Write_AccessoryInsertionOrRemovalDetected = 0x3053

const DAQmx_Write_DevsWithInsertedOrRemovedAccessories = 0x3054

const DAQmx_Write_RawDataWidth = 0x217d

const DAQmx_Write_NumChans = 0x217e

const DAQmx_Write_WaitMode = 0x22b1

const DAQmx_Write_SleepTime = 0x22b2

const DAQmx_Write_DigitalLines_BytesPerChan = 0x217f

const DAQmx_ReadWaitMode = DAQmx_Read_WaitMode

const DAQmx_Val_Task_Start = 0

const DAQmx_Val_Task_Stop = 1

const DAQmx_Val_Task_Verify = 2

const DAQmx_Val_Task_Commit = 3

const DAQmx_Val_Task_Reserve = 4

const DAQmx_Val_Task_Unreserve = 5

const DAQmx_Val_Task_Abort = 6

const DAQmx_Val_SynchronousEventCallbacks = 1 << 0

const DAQmx_Val_Acquired_Into_Buffer = 1

const DAQmx_Val_Transferred_From_Buffer = 2

const DAQmx_Val_ResetTimer = 0

const DAQmx_Val_ClearExpiration = 1

const DAQmx_Val_ChanPerLine = 0

const DAQmx_Val_ChanForAllLines = 1

const DAQmx_Val_GroupByChannel = 0

const DAQmx_Val_GroupByScanNumber = 1

const DAQmx_Val_DoNotInvertPolarity = 0

const DAQmx_Val_InvertPolarity = 1

const DAQmx_Val_Action_Commit = 0

const DAQmx_Val_Action_Cancel = 1

const DAQmx_Val_AdvanceTrigger = 12488

const DAQmx_Val_Rising = 10280

const DAQmx_Val_Falling = 10171

const DAQmx_Val_PathStatus_Available = 10431

const DAQmx_Val_PathStatus_AlreadyExists = 10432

const DAQmx_Val_PathStatus_Unsupported = 10433

const DAQmx_Val_PathStatus_ChannelInUse = 10434

const DAQmx_Val_PathStatus_SourceChannelConflict = 10435

const DAQmx_Val_PathStatus_ChannelReservedForRouting = 10436

const DAQmx_Val_DegC = 10143

const DAQmx_Val_DegF = 10144

const DAQmx_Val_Kelvins = 10325

const DAQmx_Val_DegR = 10145

const DAQmx_Val_High = 10192

const DAQmx_Val_Low = 10214

const DAQmx_Val_Tristate = 10310

const DAQmx_Val_PullUp = 15950

const DAQmx_Val_PullDown = 15951

const DAQmx_Val_ChannelVoltage = 0

const DAQmx_Val_ChannelCurrent = 1

const DAQmx_Val_ChannelHighImpedance = 2

const DAQmx_Val_Open = 10437

const DAQmx_Val_Closed = 10438

const DAQmx_Val_Loopback0 = 0

const DAQmx_Val_Loopback180 = 1

const DAQmx_Val_Ground = 2

const DAQmx_Val_Voltage = 10322

const DAQmx_Val_Bridge = 15908

const DAQmx_Val_Current = 10134

const DAQmx_Val_Diff = 10106

const DAQmx_Val_PseudoDiff = 12529

const DAQmx_Val_Charge = 16105

const DAQmx_Val_PowerCalibrationType_RemoteVoltage = 15100

const DAQmx_Val_PowerCalibrationType_LocalVoltage = 15101

const DAQmx_Val_PowerCalibrationType_Current = 15102

const DAQmx_Val_A = 12513

const DAQmx_Val_B = 12514

const DAQmx_Val_Newtons = 15875

const DAQmx_Val_Pounds = 15876

const DAQmx_Val_FromCustomScale = 10065

const DAQmx_Val_StartTrigger = 12491

const DAQmx_Val_ReferenceTrigger = 12490

const DAQmx_Val_ArmStartTrigger = 14641

const DAQmx_Val_FirstSampleTimestamp = 16130

const DAQmx_Val_Cfg_Default = -1

const DAQmx_Val_Default = -1

const DAQmx_Val_WaitInfinitely = -1.0

const DAQmx_Val_Auto = -1

const DAQmx_Val_Save_Overwrite = 1 << 0

const DAQmx_Val_Save_AllowInteractiveEditing = 1 << 1

const DAQmx_Val_Save_AllowInteractiveDeletion = 1 << 2

const DAQmx_Val_Bit_TriggerUsageTypes_Advance = 1 << 0

const DAQmx_Val_Bit_TriggerUsageTypes_Pause = 1 << 1

const DAQmx_Val_Bit_TriggerUsageTypes_Reference = 1 << 2

const DAQmx_Val_Bit_TriggerUsageTypes_Start = 1 << 3

const DAQmx_Val_Bit_TriggerUsageTypes_Handshake = 1 << 4

const DAQmx_Val_Bit_TriggerUsageTypes_ArmStart = 1 << 5

const DAQmx_Val_Bit_CouplingTypes_AC = 1 << 0

const DAQmx_Val_Bit_CouplingTypes_DC = 1 << 1

const DAQmx_Val_Bit_CouplingTypes_Ground = 1 << 2

const DAQmx_Val_Bit_CouplingTypes_HFReject = 1 << 3

const DAQmx_Val_Bit_CouplingTypes_LFReject = 1 << 4

const DAQmx_Val_Bit_CouplingTypes_NoiseReject = 1 << 5

const DAQmx_Val_Bit_TermCfg_RSE = 1 << 0

const DAQmx_Val_Bit_TermCfg_NRSE = 1 << 1

const DAQmx_Val_Bit_TermCfg_Diff = 1 << 2

const DAQmx_Val_Bit_TermCfg_PseudoDIFF = 1 << 3

const DAQmx_Val_4Wire = 4

const DAQmx_Val_5Wire = 5

const DAQmx_Val_6Wire = 6

const DAQmx_Val_Automatic = 16097

const DAQmx_Val_HighResolution = 10195

const DAQmx_Val_HighSpeed = 14712

const DAQmx_Val_Best50HzRejection = 14713

const DAQmx_Val_Best60HzRejection = 14714

const DAQmx_Val_Custom = 10137

const DAQmx_Val_VoltageRMS = 10350

const DAQmx_Val_CurrentRMS = 10351

const DAQmx_Val_Voltage_CustomWithExcitation = 10323

const DAQmx_Val_Freq_Voltage = 10181

const DAQmx_Val_Resistance = 10278

const DAQmx_Val_Temp_TC = 10303

const DAQmx_Val_Temp_Thrmstr = 10302

const DAQmx_Val_Temp_RTD = 10301

const DAQmx_Val_Temp_BuiltInSensor = 10311

const DAQmx_Val_Strain_Gage = 10300

const DAQmx_Val_Rosette_Strain_Gage = 15980

const DAQmx_Val_Position_LVDT = 10352

const DAQmx_Val_Position_RVDT = 10353

const DAQmx_Val_Position_EddyCurrentProximityProbe = 14835

const DAQmx_Val_Accelerometer = 10356

const DAQmx_Val_Acceleration_Charge = 16104

const DAQmx_Val_Acceleration_4WireDCVoltage = 16106

const DAQmx_Val_Velocity_IEPESensor = 15966

const DAQmx_Val_Force_Bridge = 15899

const DAQmx_Val_Force_IEPESensor = 15895

const DAQmx_Val_Pressure_Bridge = 15902

const DAQmx_Val_SoundPressure_Microphone = 10354

const DAQmx_Val_Torque_Bridge = 15905

const DAQmx_Val_TEDS_Sensor = 12531

const DAQmx_Val_Power = 16201

const DAQmx_Val_ZeroVolts = 12526

const DAQmx_Val_HighImpedance = 12527

const DAQmx_Val_MaintainExistingValue = 12528

const DAQmx_Val_FuncGen = 14750

const DAQmx_Val_PicoCoulombsPerG = 16099

const DAQmx_Val_PicoCoulombsPerMetersPerSecondSquared = 16100

const DAQmx_Val_PicoCoulombsPerInchesPerSecondSquared = 16101

const DAQmx_Val_mVoltsPerG = 12509

const DAQmx_Val_VoltsPerG = 12510

const DAQmx_Val_AccelUnit_g = 10186

const DAQmx_Val_MetersPerSecondSquared = 12470

const DAQmx_Val_InchesPerSecondSquared = 12471

const DAQmx_Val_FiniteSamps = 10178

const DAQmx_Val_ContSamps = 10123

const DAQmx_Val_HWTimedSinglePoint = 12522

const DAQmx_Val_AboveLvl = 10093

const DAQmx_Val_BelowLvl = 10107

const DAQmx_Val_Degrees = 10146

const DAQmx_Val_Radians = 10273

const DAQmx_Val_Ticks = 10304

const DAQmx_Val_RPM = 16080

const DAQmx_Val_RadiansPerSecond = 16081

const DAQmx_Val_DegreesPerSecond = 16082

const DAQmx_Val_None = 10230

const DAQmx_Val_Once = 10244

const DAQmx_Val_EverySample = 10164

const DAQmx_Val_NoAction = 10227

const DAQmx_Val_BreakBeforeMake = 10110

const DAQmx_Val_FullBridge = 10182

const DAQmx_Val_HalfBridge = 10187

const DAQmx_Val_QuarterBridge = 10270

const DAQmx_Val_NoBridge = 10228

const DAQmx_Val_VoltsPerVolt = 15896

const DAQmx_Val_mVoltsPerVolt = 15897

const DAQmx_Val_KilogramForce = 15877

const DAQmx_Val_Pascals = 10081

const DAQmx_Val_PoundsPerSquareInch = 15879

const DAQmx_Val_Bar = 15880

const DAQmx_Val_NewtonMeters = 15881

const DAQmx_Val_InchOunces = 15882

const DAQmx_Val_InchPounds = 15883

const DAQmx_Val_FootPounds = 15884

const DAQmx_Val_FromTEDS = 12516

const DAQmx_Val_PCI = 12582

const DAQmx_Val_PCIe = 13612

const DAQmx_Val_PXI = 12583

const DAQmx_Val_PXIe = 14706

const DAQmx_Val_SCXI = 12584

const DAQmx_Val_SCC = 14707

const DAQmx_Val_PCCard = 12585

const DAQmx_Val_USB = 12586

const DAQmx_Val_CompactDAQ = 14637

const DAQmx_Val_CompactRIO = 16143

const DAQmx_Val_TCPIP = 14828

const DAQmx_Val_Unknown = 12588

const DAQmx_Val_SwitchBlock = 15870

const DAQmx_Val_CountEdges = 10125

const DAQmx_Val_Freq = 10179

const DAQmx_Val_Period = 10256

const DAQmx_Val_PulseWidth = 10359

const DAQmx_Val_SemiPeriod = 10289

const DAQmx_Val_PulseFrequency = 15864

const DAQmx_Val_PulseTime = 15865

const DAQmx_Val_PulseTicks = 15866

const DAQmx_Val_DutyCycle = 16070

const DAQmx_Val_Position_AngEncoder = 10360

const DAQmx_Val_Position_LinEncoder = 10361

const DAQmx_Val_Velocity_AngEncoder = 16078

const DAQmx_Val_Velocity_LinEncoder = 16079

const DAQmx_Val_TwoEdgeSep = 10267

const DAQmx_Val_GPS_Timestamp = 10362

const DAQmx_Val_BuiltIn = 10200

const DAQmx_Val_ConstVal = 10116

const DAQmx_Val_Chan = 10113

const DAQmx_Val_Pulse_Time = 10269

const DAQmx_Val_Pulse_Freq = 10119

const DAQmx_Val_Pulse_Ticks = 10268

const DAQmx_Val_AI = 10100

const DAQmx_Val_AO = 10102

const DAQmx_Val_DI = 10151

const DAQmx_Val_DO = 10153

const DAQmx_Val_CI = 10131

const DAQmx_Val_CO = 10132

const DAQmx_Val_Unconstrained = 14708

const DAQmx_Val_FixedHighFreq = 14709

const DAQmx_Val_FixedLowFreq = 14710

const DAQmx_Val_Fixed50PercentDutyCycle = 14711

const DAQmx_Val_CountUp = 10128

const DAQmx_Val_CountDown = 10124

const DAQmx_Val_ExtControlled = 10326

const DAQmx_Val_LowFreq1Ctr = 10105

const DAQmx_Val_HighFreq2Ctr = 10157

const DAQmx_Val_LargeRng2Ctr = 10205

const DAQmx_Val_DynAvg = 16065

const DAQmx_Val_AC = 10045

const DAQmx_Val_DC = 10050

const DAQmx_Val_GND = 10066

const DAQmx_Val_Internal = 10200

const DAQmx_Val_External = 10167

const DAQmx_Val_UserProvided = 10167

const DAQmx_Val_Coulombs = 16102

const DAQmx_Val_PicoCoulombs = 16103

const DAQmx_Val_Amps = 10342

const DAQmx_Val_RightJustified = 10279

const DAQmx_Val_LeftJustified = 10209

const DAQmx_Val_DMA = 10054

const DAQmx_Val_Interrupts = 10204

const DAQmx_Val_ProgrammedIO = 10264

const DAQmx_Val_USBbulk = 12590

const DAQmx_Val_OnbrdMemMoreThanHalfFull = 10237

const DAQmx_Val_OnbrdMemFull = 10236

const DAQmx_Val_OnbrdMemCustomThreshold = 12577

const DAQmx_Val_ActiveDrive = 12573

const DAQmx_Val_OpenCollector = 12574

const DAQmx_Val_NoChange = 10160

const DAQmx_Val_PatternMatches = 10254

const DAQmx_Val_PatternDoesNotMatch = 10253

const DAQmx_Val_SampClkPeriods = 10286

const DAQmx_Val_Seconds = 10364

const DAQmx_Val_SampleClkPeriods = 10286

const DAQmx_Val_mVoltsPerMil = 14836

const DAQmx_Val_VoltsPerMil = 14837

const DAQmx_Val_mVoltsPerMillimeter = 14838

const DAQmx_Val_VoltsPerMillimeter = 14839

const DAQmx_Val_mVoltsPerMicron = 14840

const DAQmx_Val_X1 = 10090

const DAQmx_Val_X2 = 10091

const DAQmx_Val_X4 = 10092

const DAQmx_Val_TwoPulseCounting = 10313

const DAQmx_Val_AHighBHigh = 10040

const DAQmx_Val_AHighBLow = 10041

const DAQmx_Val_ALowBHigh = 10042

const DAQmx_Val_ALowBLow = 10043

const DAQmx_Val_Pulse = 10265

const DAQmx_Val_Toggle = 10307

const DAQmx_Val_Lvl = 10210

const DAQmx_Val_Interlocked = 12549

const DAQmx_Val_Lowpass = 16071

const DAQmx_Val_Highpass = 16072

const DAQmx_Val_Bandpass = 16073

const DAQmx_Val_Notch = 16074

const DAQmx_Val_ConstantGroupDelay = 16075

const DAQmx_Val_Butterworth = 16076

const DAQmx_Val_Elliptical = 16077

const DAQmx_Val_HardwareDefined = 10191

const DAQmx_Val_Comb = 16152

const DAQmx_Val_Bessel = 16153

const DAQmx_Val_Brickwall = 16155

const DAQmx_Val_mVoltsPerNewton = 15891

const DAQmx_Val_mVoltsPerPound = 15892

const DAQmx_Val_Hz = 10373

const DAQmx_Val_Sine = 14751

const DAQmx_Val_Triangle = 14752

const DAQmx_Val_Square = 14753

const DAQmx_Val_Sawtooth = 14754

const DAQmx_Val_IRIGB = 10070

const DAQmx_Val_PPS = 10080

const DAQmx_Val_Immediate = 10198

const DAQmx_Val_WaitForHandshakeTriggerAssert = 12550

const DAQmx_Val_WaitForHandshakeTriggerDeassert = 12551

const DAQmx_Val_OnBrdMemMoreThanHalfFull = 10237

const DAQmx_Val_OnBrdMemNotEmpty = 10241

const DAQmx_Val_WhenAcqComplete = 12546

const DAQmx_Val_RSE = 10083

const DAQmx_Val_NRSE = 10078

const DAQmx_Val_mVoltsPerVoltPerMillimeter = 12506

const DAQmx_Val_mVoltsPerVoltPerMilliInch = 12505

const DAQmx_Val_Meters = 10219

const DAQmx_Val_Inches = 10379

const DAQmx_Val_Off = 10231

const DAQmx_Val_Log = 15844

const DAQmx_Val_LogAndRead = 15842

const DAQmx_Val_OpenOrCreate = 15846

const DAQmx_Val_CreateOrReplace = 15847

const DAQmx_Val_Create = 15848

const DAQmx_Val_2point5V = 14620

const DAQmx_Val_3point3V = 14621

const DAQmx_Val_5V = 14619

const DAQmx_Val_SameAsSampTimebase = 10284

const DAQmx_Val_SameAsMasterTimebase = 10282

const DAQmx_Val_100MHzTimebase = 15857

const DAQmx_Val_80MHzTimebase = 14636

const DAQmx_Val_20MHzTimebase = 12537

const DAQmx_Val_8MHzTimebase = 16023

const DAQmx_Val_AM = 14756

const DAQmx_Val_FM = 14757

const DAQmx_Val_OnBrdMemEmpty = 10235

const DAQmx_Val_OnBrdMemHalfFullOrLess = 10239

const DAQmx_Val_OnBrdMemNotFull = 10242

const DAQmx_Val_StopTaskAndError = 15862

const DAQmx_Val_IgnoreOverruns = 15863

const DAQmx_Val_OverwriteUnreadSamps = 10252

const DAQmx_Val_DoNotOverwriteUnreadSamps = 10159

const DAQmx_Val_ActiveHigh = 10095

const DAQmx_Val_ActiveLow = 10096

const DAQmx_Val_OutputDisabled = 15503

const DAQmx_Val_ConstantVoltage = 15500

const DAQmx_Val_ConstantCurrent = 15501

const DAQmx_Val_Overvoltage = 15502

const DAQmx_Val_MSeriesDAQ = 14643

const DAQmx_Val_XSeriesDAQ = 15858

const DAQmx_Val_ESeriesDAQ = 14642

const DAQmx_Val_SSeriesDAQ = 14644

const DAQmx_Val_BSeriesDAQ = 14662

const DAQmx_Val_SCSeriesDAQ = 14645

const DAQmx_Val_USBDAQ = 14646

const DAQmx_Val_AOSeries = 14647

const DAQmx_Val_DigitalIO = 14648

const DAQmx_Val_TIOSeries = 14661

const DAQmx_Val_DynamicSignalAcquisition = 14649

const DAQmx_Val_Switches = 14650

const DAQmx_Val_CompactDAQChassis = 14658

const DAQmx_Val_CompactRIOChassis = 16144

const DAQmx_Val_CSeriesModule = 14659

const DAQmx_Val_SCXIModule = 14660

const DAQmx_Val_SCCConnectorBlock = 14704

const DAQmx_Val_SCCModule = 14705

const DAQmx_Val_NIELVIS = 14755

const DAQmx_Val_NetworkDAQ = 14829

const DAQmx_Val_SCExpress = 15886

const DAQmx_Val_FieldDAQ = 16151

const DAQmx_Val_TestScaleChassis = 16180

const DAQmx_Val_TestScaleModule = 16181

const DAQmx_Val_Pt3750 = 12481

const DAQmx_Val_Pt3851 = 10071

const DAQmx_Val_Pt3911 = 12482

const DAQmx_Val_Pt3916 = 10069

const DAQmx_Val_Pt3920 = 10053

const DAQmx_Val_Pt3928 = 12483

const DAQmx_Val_mVoltsPerVoltPerDegree = 12507

const DAQmx_Val_mVoltsPerVoltPerRadian = 12508

const DAQmx_Val_LosslessPacking = 12555

const DAQmx_Val_LossyLSBRemoval = 12556

const DAQmx_Val_FirstSample = 10424

const DAQmx_Val_CurrReadPos = 10425

const DAQmx_Val_RefTrig = 10426

const DAQmx_Val_FirstPretrigSamp = 10427

const DAQmx_Val_MostRecentSamp = 10428

const DAQmx_Val_AllowRegen = 10097

const DAQmx_Val_DoNotAllowRegen = 10158

const DAQmx_Val_2Wire = 2

const DAQmx_Val_3Wire = 3

const DAQmx_Val_Ohms = 10384

const DAQmx_Val_Bits = 10109

const DAQmx_Val_SCXI1124Range0to1V = 14629

const DAQmx_Val_SCXI1124Range0to5V = 14630

const DAQmx_Val_SCXI1124Range0to10V = 14631

const DAQmx_Val_SCXI1124RangeNeg1to1V = 14632

const DAQmx_Val_SCXI1124RangeNeg5to5V = 14633

const DAQmx_Val_SCXI1124RangeNeg10to10V = 14634

const DAQmx_Val_SCXI1124Range0to20mA = 14635

const DAQmx_Val_SampClkActiveEdge = 14617

const DAQmx_Val_SampClkInactiveEdge = 14618

const DAQmx_Val_HandshakeTriggerAsserts = 12552

const DAQmx_Val_HandshakeTriggerDeasserts = 12553

const DAQmx_Val_SampClk = 10388

const DAQmx_Val_BurstHandshake = 12548

const DAQmx_Val_Handshake = 10389

const DAQmx_Val_Implicit = 10451

const DAQmx_Val_OnDemand = 10390

const DAQmx_Val_ChangeDetection = 12504

const DAQmx_Val_PipelinedSampClk = 14668

const DAQmx_Val_Linear = 10447

const DAQmx_Val_MapRanges = 10448

const DAQmx_Val_Polynomial = 10449

const DAQmx_Val_Table = 10450

const DAQmx_Val_TwoPointLinear = 15898

const DAQmx_Val_Enabled = 16145

const DAQmx_Val_Disabled = 16146

const DAQmx_Val_BipolarDC = 16147

const DAQmx_Val_AandB = 12515

const DAQmx_Val_R1 = 12465

const DAQmx_Val_R2 = 12466

const DAQmx_Val_R3 = 12467

const DAQmx_Val_R4 = 14813

const DAQmx_Val_AIConvertClock = 12484

const DAQmx_Val_10MHzRefClock = 12536

const DAQmx_Val_20MHzTimebaseClock = 12486

const DAQmx_Val_SampleClock = 12487

const DAQmx_Val_AdvCmpltEvent = 12492

const DAQmx_Val_AIHoldCmpltEvent = 12493

const DAQmx_Val_CounterOutputEvent = 12494

const DAQmx_Val_ChangeDetectionEvent = 12511

const DAQmx_Val_WDTExpiredEvent = 12512

const DAQmx_Val_SampleCompleteEvent = 12530

const DAQmx_Val_RisingSlope = 10280

const DAQmx_Val_FallingSlope = 10171

const DAQmx_Val_FullBridgeI = 10183

const DAQmx_Val_FullBridgeII = 10184

const DAQmx_Val_FullBridgeIII = 10185

const DAQmx_Val_HalfBridgeI = 10188

const DAQmx_Val_HalfBridgeII = 10189

const DAQmx_Val_QuarterBridgeI = 10271

const DAQmx_Val_QuarterBridgeII = 10272

const DAQmx_Val_RectangularRosette = 15968

const DAQmx_Val_DeltaRosette = 15969

const DAQmx_Val_TeeRosette = 15970

const DAQmx_Val_PrincipalStrain1 = 15971

const DAQmx_Val_PrincipalStrain2 = 15972

const DAQmx_Val_PrincipalStrainAngle = 15973

const DAQmx_Val_CartesianStrainX = 15974

const DAQmx_Val_CartesianStrainY = 15975

const DAQmx_Val_CartesianShearStrainXY = 15976

const DAQmx_Val_MaxShearStrain = 15977

const DAQmx_Val_MaxShearStrainAngle = 15978

const DAQmx_Val_Strain = 10299

const DAQmx_Val_Finite = 10172

const DAQmx_Val_Cont = 10117

const DAQmx_Val_Source = 10439

const DAQmx_Val_Load = 10440

const DAQmx_Val_ReservedForRouting = 10441

const DAQmx_Val_Onboard = 16128

const DAQmx_Val_DigEdge = 10150

const DAQmx_Val_Time = 15996

const DAQmx_Val_Master = 15888

const DAQmx_Val_Slave = 15889

const DAQmx_Val_IgnoreLostSyncLock = 16129

const DAQmx_Val_J_Type_TC = 10072

const DAQmx_Val_K_Type_TC = 10073

const DAQmx_Val_N_Type_TC = 10077

const DAQmx_Val_R_Type_TC = 10082

const DAQmx_Val_S_Type_TC = 10085

const DAQmx_Val_T_Type_TC = 10086

const DAQmx_Val_B_Type_TC = 10047

const DAQmx_Val_E_Type_TC = 10055

const DAQmx_Val_HostTime = 16126

const DAQmx_Val_IODeviceTime = 16127

const DAQmx_Val_SingleCycle = 14613

const DAQmx_Val_Multicycle = 14614

const DAQmx_Val_Software = 10292

const DAQmx_Val_AnlgLvl = 10101

const DAQmx_Val_AnlgWin = 10103

const DAQmx_Val_DigLvl = 10152

const DAQmx_Val_DigPattern = 10398

const DAQmx_Val_AnlgEdge = 10099

const DAQmx_Val_AnlgMultiEdge = 16108

const DAQmx_Val_HaltOutputAndError = 14615

const DAQmx_Val_PauseUntilDataAvailable = 14616

const DAQmx_Val_Volts = 10348

const DAQmx_Val_g = 10186

const DAQmx_Val_MetersPerSecond = 15959

const DAQmx_Val_InchesPerSecond = 15960

const DAQmx_Val_MillivoltsPerMillimeterPerSecond = 15963

const DAQmx_Val_MilliVoltsPerInchPerSecond = 15964

const DAQmx_Val_WaitForInterrupt = 12523

const DAQmx_Val_Poll = 12524

const DAQmx_Val_Yield = 12525

const DAQmx_Val_Sleep = 12547

const DAQmx_Val_EnteringWin = 10163

const DAQmx_Val_LeavingWin = 10208

const DAQmx_Val_InsideWin = 10199

const DAQmx_Val_OutsideWin = 10251

const DAQmx_Val_WriteToEEPROM = 12538

const DAQmx_Val_WriteToPROM = 12539

const DAQmx_Val_DoNotWrite = 12540

const DAQmx_Val_CurrWritePos = 10430

const DAQmx_Val_ZeroVoltsOrAmps = 12526

const DAQmx_Val_RepeatedData = 16062

const DAQmx_Val_SentinelValue = 16063

const DAQmx_Val_LogicLevelPullUp = 16064

const DAQmx_Val_Local = 16095

const DAQmx_Val_Remote = 16096

const DAQmx_Val_Switch_Topology_Configured_Topology = "Configured Topology"

const DAQmx_Val_Switch_Topology_1127_1_Wire_64x1_Mux = "1127/1-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_1127_2_Wire_32x1_Mux = "1127/2-Wire 32x1 Mux"

const DAQmx_Val_Switch_Topology_1127_2_Wire_4x8_Matrix = "1127/2-Wire 4x8 Matrix"

const DAQmx_Val_Switch_Topology_1127_4_Wire_16x1_Mux = "1127/4-Wire 16x1 Mux"

const DAQmx_Val_Switch_Topology_1127_Independent = "1127/Independent"

const DAQmx_Val_Switch_Topology_1128_1_Wire_64x1_Mux = "1128/1-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_1128_2_Wire_32x1_Mux = "1128/2-Wire 32x1 Mux"

const DAQmx_Val_Switch_Topology_1128_2_Wire_4x8_Matrix = "1128/2-Wire 4x8 Matrix"

const DAQmx_Val_Switch_Topology_1128_4_Wire_16x1_Mux = "1128/4-Wire 16x1 Mux"

const DAQmx_Val_Switch_Topology_1128_Independent = "1128/Independent"

const DAQmx_Val_Switch_Topology_1129_2_Wire_16x16_Matrix = "1129/2-Wire 16x16 Matrix"

const DAQmx_Val_Switch_Topology_1129_2_Wire_8x32_Matrix = "1129/2-Wire 8x32 Matrix"

const DAQmx_Val_Switch_Topology_1129_2_Wire_4x64_Matrix = "1129/2-Wire 4x64 Matrix"

const DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_8x16_Matrix = "1129/2-Wire Dual 8x16 Matrix"

const DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_4x32_Matrix = "1129/2-Wire Dual 4x32 Matrix"

const DAQmx_Val_Switch_Topology_1129_2_Wire_Quad_4x16_Matrix = "1129/2-Wire Quad 4x16 Matrix"

const DAQmx_Val_Switch_Topology_1130_1_Wire_256x1_Mux = "1130/1-Wire 256x1 Mux"

const DAQmx_Val_Switch_Topology_1130_1_Wire_Dual_128x1_Mux = "1130/1-Wire Dual 128x1 Mux"

const DAQmx_Val_Switch_Topology_1130_2_Wire_128x1_Mux = "1130/2-Wire 128x1 Mux"

const DAQmx_Val_Switch_Topology_1130_4_Wire_64x1_Mux = "1130/4-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_1130_1_Wire_4x64_Matrix = "1130/1-Wire 4x64 Matrix"

const DAQmx_Val_Switch_Topology_1130_1_Wire_8x32_Matrix = "1130/1-Wire 8x32 Matrix"

const DAQmx_Val_Switch_Topology_1130_1_Wire_Octal_32x1_Mux = "1130/1-Wire Octal 32x1 Mux"

const DAQmx_Val_Switch_Topology_1130_1_Wire_Quad_64x1_Mux = "1130/1-Wire Quad 64x1 Mux"

const DAQmx_Val_Switch_Topology_1130_1_Wire_Sixteen_16x1_Mux = "1130/1-Wire Sixteen 16x1 Mux"

const DAQmx_Val_Switch_Topology_1130_2_Wire_4x32_Matrix = "1130/2-Wire 4x32 Matrix"

const DAQmx_Val_Switch_Topology_1130_2_Wire_Octal_16x1_Mux = "1130/2-Wire Octal 16x1 Mux"

const DAQmx_Val_Switch_Topology_1130_2_Wire_Quad_32x1_Mux = "1130/2-Wire Quad 32x1 Mux"

const DAQmx_Val_Switch_Topology_1130_4_Wire_Quad_16x1_Mux = "1130/4-Wire Quad 16x1 Mux"

const DAQmx_Val_Switch_Topology_1130_Independent = "1130/Independent"

const DAQmx_Val_Switch_Topology_1160_16_SPDT = "1160/16-SPDT"

const DAQmx_Val_Switch_Topology_1161_8_SPDT = "1161/8-SPDT"

const DAQmx_Val_Switch_Topology_1163R_Octal_4x1_Mux = "1163R/Octal 4x1 Mux"

const DAQmx_Val_Switch_Topology_1166_32_SPDT = "1166/32-SPDT"

const DAQmx_Val_Switch_Topology_1166_16_DPDT = "1166/16-DPDT"

const DAQmx_Val_Switch_Topology_1167_Independent = "1167/Independent"

const DAQmx_Val_Switch_Topology_1169_100_SPST = "1169/100-SPST"

const DAQmx_Val_Switch_Topology_1169_50_DPST = "1169/50-DPST"

const DAQmx_Val_Switch_Topology_1175_1_Wire_196x1_Mux = "1175/1-Wire 196x1 Mux"

const DAQmx_Val_Switch_Topology_1175_2_Wire_98x1_Mux = "1175/2-Wire 98x1 Mux"

const DAQmx_Val_Switch_Topology_1175_2_Wire_95x1_Mux = "1175/2-Wire 95x1 Mux"

const DAQmx_Val_Switch_Topology_1190_Quad_4x1_Mux = "1190/Quad 4x1 Mux"

const DAQmx_Val_Switch_Topology_1191_Quad_4x1_Mux = "1191/Quad 4x1 Mux"

const DAQmx_Val_Switch_Topology_1192_8_SPDT = "1192/8-SPDT"

const DAQmx_Val_Switch_Topology_1193_32x1_Mux = "1193/32x1 Mux"

const DAQmx_Val_Switch_Topology_1193_Dual_16x1_Mux = "1193/Dual 16x1 Mux"

const DAQmx_Val_Switch_Topology_1193_Quad_8x1_Mux = "1193/Quad 8x1 Mux"

const DAQmx_Val_Switch_Topology_1193_16x1_Terminated_Mux = "1193/16x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_1193_Dual_8x1_Terminated_Mux = "1193/Dual 8x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_1193_Quad_4x1_Terminated_Mux = "1193/Quad 4x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_1193_Independent = "1193/Independent"

const DAQmx_Val_Switch_Topology_1194_Quad_4x1_Mux = "1194/Quad 4x1 Mux"

const DAQmx_Val_Switch_Topology_1195_Quad_4x1_Mux = "1195/Quad 4x1 Mux"

const DAQmx_Val_Switch_Topology_2501_1_Wire_48x1_Mux = "2501/1-Wire 48x1 Mux"

const DAQmx_Val_Switch_Topology_2501_1_Wire_48x1_Amplified_Mux = "2501/1-Wire 48x1 Amplified Mux"

const DAQmx_Val_Switch_Topology_2501_2_Wire_24x1_Mux = "2501/2-Wire 24x1 Mux"

const DAQmx_Val_Switch_Topology_2501_2_Wire_24x1_Amplified_Mux = "2501/2-Wire 24x1 Amplified Mux"

const DAQmx_Val_Switch_Topology_2501_2_Wire_Dual_12x1_Mux = "2501/2-Wire Dual 12x1 Mux"

const DAQmx_Val_Switch_Topology_2501_2_Wire_Quad_6x1_Mux = "2501/2-Wire Quad 6x1 Mux"

const DAQmx_Val_Switch_Topology_2501_2_Wire_4x6_Matrix = "2501/2-Wire 4x6 Matrix"

const DAQmx_Val_Switch_Topology_2501_4_Wire_12x1_Mux = "2501/4-Wire 12x1 Mux"

const DAQmx_Val_Switch_Topology_2503_1_Wire_48x1_Mux = "2503/1-Wire 48x1 Mux"

const DAQmx_Val_Switch_Topology_2503_2_Wire_24x1_Mux = "2503/2-Wire 24x1 Mux"

const DAQmx_Val_Switch_Topology_2503_2_Wire_Dual_12x1_Mux = "2503/2-Wire Dual 12x1 Mux"

const DAQmx_Val_Switch_Topology_2503_2_Wire_Quad_6x1_Mux = "2503/2-Wire Quad 6x1 Mux"

const DAQmx_Val_Switch_Topology_2503_2_Wire_4x6_Matrix = "2503/2-Wire 4x6 Matrix"

const DAQmx_Val_Switch_Topology_2503_4_Wire_12x1_Mux = "2503/4-Wire 12x1 Mux"

const DAQmx_Val_Switch_Topology_2510_Independent = "2510/Independent"

const DAQmx_Val_Switch_Topology_2512_Independent = "2512/Independent"

const DAQmx_Val_Switch_Topology_2514_Independent = "2514/Independent"

const DAQmx_Val_Switch_Topology_2515_Independent = "2515/Independent"

const DAQmx_Val_Switch_Topology_2520_80_SPST = "2520/80-SPST"

const DAQmx_Val_Switch_Topology_2521_40_DPST = "2521/40-DPST"

const DAQmx_Val_Switch_Topology_2522_53_SPDT = "2522/53-SPDT"

const DAQmx_Val_Switch_Topology_2523_26_DPDT = "2523/26-DPDT"

const DAQmx_Val_Switch_Topology_2527_1_Wire_64x1_Mux = "2527/1-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_2527_1_Wire_Dual_32x1_Mux = "2527/1-Wire Dual 32x1 Mux"

const DAQmx_Val_Switch_Topology_2527_2_Wire_32x1_Mux = "2527/2-Wire 32x1 Mux"

const DAQmx_Val_Switch_Topology_2527_2_Wire_Dual_16x1_Mux = "2527/2-Wire Dual 16x1 Mux"

const DAQmx_Val_Switch_Topology_2527_4_Wire_16x1_Mux = "2527/4-Wire 16x1 Mux"

const DAQmx_Val_Switch_Topology_2527_Independent = "2527/Independent"

const DAQmx_Val_Switch_Topology_2529_2_Wire_8x16_Matrix = "2529/2-Wire 8x16 Matrix"

const DAQmx_Val_Switch_Topology_2529_2_Wire_4x32_Matrix = "2529/2-Wire 4x32 Matrix"

const DAQmx_Val_Switch_Topology_2529_2_Wire_Dual_4x16_Matrix = "2529/2-Wire Dual 4x16 Matrix"

const DAQmx_Val_Switch_Topology_2530_1_Wire_128x1_Mux = "2530/1-Wire 128x1 Mux"

const DAQmx_Val_Switch_Topology_2530_1_Wire_Dual_64x1_Mux = "2530/1-Wire Dual 64x1 Mux"

const DAQmx_Val_Switch_Topology_2530_2_Wire_64x1_Mux = "2530/2-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_2530_4_Wire_32x1_Mux = "2530/4-Wire 32x1 Mux"

const DAQmx_Val_Switch_Topology_2530_1_Wire_4x32_Matrix = "2530/1-Wire 4x32 Matrix"

const DAQmx_Val_Switch_Topology_2530_1_Wire_8x16_Matrix = "2530/1-Wire 8x16 Matrix"

const DAQmx_Val_Switch_Topology_2530_1_Wire_Octal_16x1_Mux = "2530/1-Wire Octal 16x1 Mux"

const DAQmx_Val_Switch_Topology_2530_1_Wire_Quad_32x1_Mux = "2530/1-Wire Quad 32x1 Mux"

const DAQmx_Val_Switch_Topology_2530_2_Wire_4x16_Matrix = "2530/2-Wire 4x16 Matrix"

const DAQmx_Val_Switch_Topology_2530_2_Wire_Dual_32x1_Mux = "2530/2-Wire Dual 32x1 Mux"

const DAQmx_Val_Switch_Topology_2530_2_Wire_Quad_16x1_Mux = "2530/2-Wire Quad 16x1 Mux"

const DAQmx_Val_Switch_Topology_2530_4_Wire_Dual_16x1_Mux = "2530/4-Wire Dual 16x1 Mux"

const DAQmx_Val_Switch_Topology_2530_Independent = "2530/Independent"

const DAQmx_Val_Switch_Topology_2531_1_Wire_4x128_Matrix = "2531/1-Wire 4x128 Matrix"

const DAQmx_Val_Switch_Topology_2531_1_Wire_8x64_Matrix = "2531/1-Wire 8x64 Matrix"

const DAQmx_Val_Switch_Topology_2531_1_Wire_Dual_4x64_Matrix = "2531/1-Wire Dual 4x64 Matrix"

const DAQmx_Val_Switch_Topology_2531_1_Wire_Dual_8x32_Matrix = "2531/1-Wire Dual 8x32 Matrix"

const DAQmx_Val_Switch_Topology_2531_1_Wire_Sixteen_2x16_Matrix = "2531/1-Wire Sixteen 2x16 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_16x32_Matrix = "2532/1-Wire 16x32 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_4x128_Matrix = "2532/1-Wire 4x128 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_8x64_Matrix = "2532/1-Wire 8x64 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_16x16_Matrix = "2532/1-Wire Dual 16x16 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_4x64_Matrix = "2532/1-Wire Dual 4x64 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_Dual_8x32_Matrix = "2532/1-Wire Dual 8x32 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_Quad_4x32_Matrix = "2532/1-Wire Quad 4x32 Matrix"

const DAQmx_Val_Switch_Topology_2532_1_Wire_Sixteen_2x16_Matrix = "2532/1-Wire Sixteen 2x16 Matrix"

const DAQmx_Val_Switch_Topology_2532_2_Wire_16x16_Matrix = "2532/2-Wire 16x16 Matrix"

const DAQmx_Val_Switch_Topology_2532_2_Wire_4x64_Matrix = "2532/2-Wire 4x64 Matrix"

const DAQmx_Val_Switch_Topology_2532_2_Wire_8x32_Matrix = "2532/2-Wire 8x32 Matrix"

const DAQmx_Val_Switch_Topology_2532_2_Wire_Dual_4x32_Matrix = "2532/2-Wire Dual 4x32 Matrix"

const DAQmx_Val_Switch_Topology_2533_1_Wire_4x64_Matrix = "2533/1-Wire 4x64 Matrix"

const DAQmx_Val_Switch_Topology_2534_1_Wire_8x32_Matrix = "2534/1-Wire 8x32 Matrix"

const DAQmx_Val_Switch_Topology_2535_1_Wire_4x136_Matrix = "2535/1-Wire 4x136 Matrix"

const DAQmx_Val_Switch_Topology_2536_1_Wire_8x68_Matrix = "2536/1-Wire 8x68 Matrix"

const DAQmx_Val_Switch_Topology_2540_1_Wire_8x9_Matrix = "2540/1-Wire 8x9 Matrix"

const DAQmx_Val_Switch_Topology_2541_1_Wire_8x12_Matrix = "2541/1-Wire 8x12 Matrix"

const DAQmx_Val_Switch_Topology_2542_Quad_2x1_Terminated_Mux = "2542/Quad 2x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2543_Dual_4x1_Terminated_Mux = "2543/Dual 4x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2544_8x1_Terminated_Mux = "2544/8x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2545_4x1_Terminated_Mux = "2545/4x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2546_Dual_4x1_Mux = "2546/Dual 4x1 Mux"

const DAQmx_Val_Switch_Topology_2547_8x1_Mux = "2547/8x1 Mux"

const DAQmx_Val_Switch_Topology_2548_4_SPDT = "2548/4-SPDT"

const DAQmx_Val_Switch_Topology_2549_Terminated_2_SPDT = "2549/Terminated 2-SPDT"

const DAQmx_Val_Switch_Topology_2554_4x1_Mux = "2554/4x1 Mux"

const DAQmx_Val_Switch_Topology_2555_4x1_Terminated_Mux = "2555/4x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2556_Dual_4x1_Mux = "2556/Dual 4x1 Mux"

const DAQmx_Val_Switch_Topology_2557_8x1_Mux = "2557/8x1 Mux"

const DAQmx_Val_Switch_Topology_2558_4_SPDT = "2558/4-SPDT"

const DAQmx_Val_Switch_Topology_2559_Terminated_2_SPDT = "2559/Terminated 2-SPDT"

const DAQmx_Val_Switch_Topology_2564_16_SPST = "2564/16-SPST"

const DAQmx_Val_Switch_Topology_2564_8_DPST = "2564/8-DPST"

const DAQmx_Val_Switch_Topology_2565_16_SPST = "2565/16-SPST"

const DAQmx_Val_Switch_Topology_2566_16_SPDT = "2566/16-SPDT"

const DAQmx_Val_Switch_Topology_2566_8_DPDT = "2566/8-DPDT"

const DAQmx_Val_Switch_Topology_2567_Independent = "2567/Independent"

const DAQmx_Val_Switch_Topology_2568_31_SPST = "2568/31-SPST"

const DAQmx_Val_Switch_Topology_2568_15_DPST = "2568/15-DPST"

const DAQmx_Val_Switch_Topology_2569_100_SPST = "2569/100-SPST"

const DAQmx_Val_Switch_Topology_2569_50_DPST = "2569/50-DPST"

const DAQmx_Val_Switch_Topology_2570_40_SPDT = "2570/40-SPDT"

const DAQmx_Val_Switch_Topology_2570_20_DPDT = "2570/20-DPDT"

const DAQmx_Val_Switch_Topology_2571_66_SPDT = "2571/66-SPDT"

const DAQmx_Val_Switch_Topology_2575_1_Wire_196x1_Mux = "2575/1-Wire 196x1 Mux"

const DAQmx_Val_Switch_Topology_2575_2_Wire_98x1_Mux = "2575/2-Wire 98x1 Mux"

const DAQmx_Val_Switch_Topology_2575_2_Wire_95x1_Mux = "2575/2-Wire 95x1 Mux"

const DAQmx_Val_Switch_Topology_2576_2_Wire_64x1_Mux = "2576/2-Wire 64x1 Mux"

const DAQmx_Val_Switch_Topology_2576_2_Wire_Dual_32x1_Mux = "2576/2-Wire Dual 32x1 Mux"

const DAQmx_Val_Switch_Topology_2576_2_Wire_Octal_8x1_Mux = "2576/2-Wire Octal 8x1 Mux"

const DAQmx_Val_Switch_Topology_2576_2_Wire_Quad_16x1_Mux = "2576/2-Wire Quad 16x1 Mux"

const DAQmx_Val_Switch_Topology_2576_2_Wire_Sixteen_4x1_Mux = "2576/2-Wire Sixteen 4x1 Mux"

const DAQmx_Val_Switch_Topology_2576_Independent = "2576/Independent"

const DAQmx_Val_Switch_Topology_2584_1_Wire_12x1_Mux = "2584/1-Wire 12x1 Mux"

const DAQmx_Val_Switch_Topology_2584_1_Wire_Dual_6x1_Mux = "2584/1-Wire Dual 6x1 Mux"

const DAQmx_Val_Switch_Topology_2584_2_Wire_6x1_Mux = "2584/2-Wire 6x1 Mux"

const DAQmx_Val_Switch_Topology_2584_Independent = "2584/Independent"

const DAQmx_Val_Switch_Topology_2585_1_Wire_10x1_Mux = "2585/1-Wire 10x1 Mux"

const DAQmx_Val_Switch_Topology_2586_10_SPST = "2586/10-SPST"

const DAQmx_Val_Switch_Topology_2586_5_DPST = "2586/5-DPST"

const DAQmx_Val_Switch_Topology_2590_4x1_Mux = "2590/4x1 Mux"

const DAQmx_Val_Switch_Topology_2591_4x1_Mux = "2591/4x1 Mux"

const DAQmx_Val_Switch_Topology_2593_16x1_Mux = "2593/16x1 Mux"

const DAQmx_Val_Switch_Topology_2593_Dual_8x1_Mux = "2593/Dual 8x1 Mux"

const DAQmx_Val_Switch_Topology_2593_8x1_Terminated_Mux = "2593/8x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2593_Dual_4x1_Terminated_Mux = "2593/Dual 4x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2593_Independent = "2593/Independent"

const DAQmx_Val_Switch_Topology_2594_4x1_Mux = "2594/4x1 Mux"

const DAQmx_Val_Switch_Topology_2595_4x1_Mux = "2595/4x1 Mux"

const DAQmx_Val_Switch_Topology_2596_Dual_6x1_Mux = "2596/Dual 6x1 Mux"

const DAQmx_Val_Switch_Topology_2597_6x1_Terminated_Mux = "2597/6x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2598_Dual_Transfer = "2598/Dual Transfer"

const DAQmx_Val_Switch_Topology_2599_2_SPDT = "2599/2-SPDT"

const DAQmx_Val_Switch_Topology_2720_Independent = "2720/Independent"

const DAQmx_Val_Switch_Topology_2722_Independent = "2722/Independent"

const DAQmx_Val_Switch_Topology_2725_Independent = "2725/Independent"

const DAQmx_Val_Switch_Topology_2727_Independent = "2727/Independent"

const DAQmx_Val_Switch_Topology_2790_Independent = "2790/Independent"

const DAQmx_Val_Switch_Topology_2796_Dual_6x1_Mux = "2796/Dual 6x1 Mux"

const DAQmx_Val_Switch_Topology_2797_6x1_Terminated_Mux = "2797/6x1 Terminated Mux"

const DAQmx_Val_Switch_Topology_2798_Dual_Transfer = "2798/Dual Transfer"

const DAQmx_Val_Switch_Topology_2799_2_SPDT = "2799/2-SPDT"

const DAQmxSuccess = 0

const DAQmxErrorRemoteSense = -209888

const DAQmxErrorOverTemperatureProtectionActivated = -209887

const DAQmxErrorMultiTaskCfgSampRateNotSupportedWithPropSet = -209886

const DAQmxErrorMultiTaskCfgSampRateConflictingProp = -209885

const DAQmxErrorNoCommonSampRateFoundNoRepeatSamps = -209884

const DAQmxErrorNoCommonSampRateFound = -209883

const DAQmxErrorMultiTaskCfgDoesNotSupportMultiDevTask = -209882

const DAQmxErrorMultiTaskSampRateCfgNotSupported = -209881

const DAQmxErrorDebugSessionNotAllowedTimingSourceRegistered = -209880

const DAQmxErrorDebugSessionNotAllowedWhenLogging = -209879

const DAQmxErrorDebugSessionNotAllowedEventRegistered = -209878

const DAQmxErrorInvalidTargetTaskForDebugSession = -209877

const DAQmxErrorFunctionNotSupportedForDevice = -209876

const DAQmxErrorMultipleTargetTasksFoundForDebugSession = -209875

const DAQmxErrorTargetTaskNotFoundForDebugSession = -209874

const DAQmxErrorOperationNotSupportedInDebugSession = -209873

const DAQmxErrorOperationNotPermittedInMonitorModeForDebugSession = -209872

const DAQmxErrorGetActiveDevPrptyFailedDueToDifftVals = -209871

const DAQmxErrorTaskAlreadyRegisteredATimingSource = -209870

const DAQmxErrorFilterNotSupportedOnHWRev = -209869

const DAQmxErrorSensorPowerSupplyVoltageLevel = -209868

const DAQmxErrorSensorPowerSupply = -209867

const DAQmxErrorInvalidScanlist = -209866

const DAQmxErrorTimeResourceCannotBeRouted = -209865

const DAQmxErrorInvalidResetDelayRequested = -209864

const DAQmxErrorExceededTotalTimetriggersAvailable = -209863

const DAQmxErrorExceededTotalTimestampsAvailable = -209862

const DAQmxErrorNoSynchronizationProtocolRunning = -209861

const DAQmxErrorConflictingCoherencyRequirements = -209860

const DAQmxErrorNoSharedTimescale = -209859

const DAQmxErrorInvalidFieldDAQBankName = -209858

const DAQmxErrorDeviceDoesNotSupportHWTSP = -209857

const DAQmxErrorBankTypeDoesNotMatchBankTypeInDestination = -209856

const DAQmxErrorInvalidFieldDAQBankNumberSpecd = -209855

const DAQmxErrorUnsupportedSimulatedBankForSimulatedFieldDAQ = -209854

const DAQmxErrorFieldDAQBankSimMustMatchFieldDAQSim = -209853

const DAQmxErrorDevNoLongerSupportedWithinDAQmxAPI = -209852

const DAQmxErrorTimingEngineDoesNotSupportOnBoardMemory = -209851

const DAQmxErrorDuplicateTaskCrossProject = -209850

const DAQmxErrorTimeStartTriggerBeforeArmStartTrigger = -209849

const DAQmxErrorTimeTriggerCannotBeSet = -209848

const DAQmxErrorInvalidTriggerWindowValue = -209847

const DAQmxErrorCannotQueryPropertyBeforeOrDuringAcquisition = -209846

const DAQmxErrorSampleClockTimebaseNotSupported = -209845

const DAQmxErrorTimestampNotYetReceived = -209844

const DAQmxErrorTimeTriggerNotSupported = -209843

const DAQmxErrorTimestampNotEnabled = -209842

const DAQmxErrorTimeTriggersInconsistent = -209841

const DAQmxErrorTriggerConfiguredIsInThePast = -209840

const DAQmxErrorTriggerConfiguredIsTooFarFromCurrentTime = -209839

const DAQmxErrorSynchronizationLockLost = -209838

const DAQmxErrorInconsistentTimescales = -209837

const DAQmxErrorCannotSynchronizeDevices = -209836

const DAQmxErrorAssociatedChansHaveAttributeConflictWithMultipleMaxMinRanges = -209835

const DAQmxErrorSampleRateNumChansOrAttributeValues = -209834

const DAQmxErrorWaitForValidTimestampNotSupported = -209833

const DAQmxErrorTrigWinTimeoutExpired = -209832

const DAQmxErrorInvalidTriggerCfgForDevice = -209831

const DAQmxErrorInvalidDataTransferMechanismForDevice = -209830

const DAQmxErrorInputFIFOOverflow3 = -209829

const DAQmxErrorTooManyDevicesForAnalogMultiEdgeTrigCDAQ = -209828

const DAQmxErrorTooManyTriggersTypesSpecifiedInTask = -209827

const DAQmxErrorMismatchedMultiTriggerConfigValues = -209826

const DAQmxErrorInconsistentAODACRangeAcrossTasks = -209825

const DAQmxErrorInconsistentDTToWrite = -209824

const DAQmxErrorFunctionObsolete = -209823

const DAQmxErrorNegativeDurationNotSupported = -209822

const DAQmxErrorDurationTooSmall = -209821

const DAQmxErrorDurationTooLong = -209820

const DAQmxErrorDurationBasedNotSupportedForSpecifiedTimingMode = -209819

const DAQmxErrorInvalidLEDState = -209818

const DAQmxErrorWatchdogStatesNotUniform = -209817

const DAQmxErrorSelfTestFailedPowerSupplyOutOfTolerance = -209816

const DAQmxErrorHWTSPMultiSampleWrite = -209815

const DAQmxErrorOnboardRegenExceedsChannelLimit = -209814

const DAQmxErrorWatchdogChannelExpirationStateNotSpecified = -209813

const DAQmxErrorInvalidShuntSourceForCalibration = -209812

const DAQmxErrorInvalidShuntSelectForCalibration = -209811

const DAQmxErrorInvalidShuntCalibrationConfiguration = -209810

const DAQmxErrorBufferedOperationsNotSupportedOnChannelStandalone = -209809

const DAQmxErrorFeatureNotAvailableOnAccessory = -209808

const DAQmxErrorInconsistentThreshVoltageAcrossTerminals = -209807

const DAQmxErrorDAQmxIsNotInstalledOnTarget = -209806

const DAQmxErrorCOCannotKeepUpInHWTimedSinglePoint = -209805

const DAQmxErrorWaitForNextSampClkDetected3OrMoreSampClks = -209803

const DAQmxErrorWaitForNextSampClkDetectedMissedSampClk = -209802

const DAQmxErrorWriteNotCompleteBeforeSampClk = -209801

const DAQmxErrorReadNotCompleteBeforeSampClk = -209800

const DAQmxErrorInconsistentDigitalFilteringAcrossTerminals = -201510

const DAQmxErrorInconsistentPullUpCfgAcrossTerminals = -201509

const DAQmxErrorInconsistentTermCfgAcrossTerminals = -201508

const DAQmxErrorVCXODCMBecameUnlocked = -201507

const DAQmxErrorPLLDACUpdateFailed = -201506

const DAQmxErrorNoCabledDevice = -201505

const DAQmxErrorLostRefClk = -201504

const DAQmxErrorCantUseAITimingEngineWithCounters = -201503

const DAQmxErrorDACOffsetValNotSet = -201502

const DAQmxErrorCalAdjustRefValOutOfRange = -201501

const DAQmxErrorChansForCalAdjustMustPerformSetContext = -201500

const DAQmxErrorGetCalDataInvalidForCalMode = -201499

const DAQmxErrorNoIEPEWithACNotAllowed = -201498

const DAQmxErrorSetupCalNeededBeforeGetCalDataPoints = -201497

const DAQmxErrorVoltageNotCalibrated = -201496

const DAQmxErrorMissingRangeForCalibration = -201495

const DAQmxErrorMultipleChansNotSupportedDuringCalAdjust = -201494

const DAQmxErrorShuntCalFailedOutOfRange = -201493

const DAQmxErrorOperationNotSupportedOnSimulatedDevice = -201492

const DAQmxErrorFirmwareVersionSameAsInstalledVersion = -201491

const DAQmxErrorFirmwareVersionOlderThanInstalledVersion = -201490

const DAQmxErrorFirmwareUpdateInvalidState = -201489

const DAQmxErrorFirmwareUpdateInvalidID = -201488

const DAQmxErrorFirmwareUpdateAutomaticManagementEnabled = -201487

const DAQmxErrorSetupCalibrationNotCalled = -201486

const DAQmxErrorCalMeasuredDataSizeVsActualDataSizeMismatch = -201485

const DAQmxErrorCDAQMissingDSAMasterForChanExpansion = -201484

const DAQmxErrorCDAQMasterNotFoundForChanExpansion = -201483

const DAQmxErrorAllChansShouldBeProvidedForCalibration = -201482

const DAQmxErrorMustSpecifyExpirationStateForAllLinesInRange = -201481

const DAQmxErrorOpenSessionExists = -201480

const DAQmxErrorCannotQueryTerminalForSWArmStart = -201479

const DAQmxErrorChassisWatchdogTimerExpired = -201478

const DAQmxErrorCantReserveWatchdogTaskWhileOtherTasksReserved = -201477

const DAQmxErrorCantReserveTaskWhileWatchdogTaskReserving = -201476

const DAQmxErrorAuxPowerSourceRequired = -201475

const DAQmxErrorDeviceNotSupportedOnLocalSystem = -201474

const DAQmxErrorOneTimestampChannelRequiredForCombinedNavigationRead = -201472

const DAQmxErrorMultDevsMultPhysChans = -201471

const DAQmxErrorInvalidCalAdjustmentPointValues = -201470

const DAQmxErrorDifferentDigitizerFromCommunicator = -201469

const DAQmxErrorCDAQSyncMasterClockNotPresent = -201468

const DAQmxErrorAssociatedChansHaveConflictingProps = -201467

const DAQmxErrorAutoConfigBetweenMultipleDeviceStatesInvalid = -201466

const DAQmxErrorAutoConfigOfOfflineDevicesInvalid = -201465

const DAQmxErrorExternalFIFOFault = -201464

const DAQmxErrorConnectionsNotReciprocal = -201463

const DAQmxErrorInvalidOutputToInputCDAQSyncConnection = -201462

const DAQmxErrorReferenceClockNotPresent = -201461

const DAQmxErrorBlankStringExpansionFoundNoSupportedCDAQSyncConnectionDevices = -201460

const DAQmxErrorNoDevicesSupportCDAQSyncConnections = -201459

const DAQmxErrorInvalidCDAQSyncTimeoutValue = -201458

const DAQmxErrorCDAQSyncConnectionToSamePort = -201457

const DAQmxErrorDevsWithoutCommonSyncConnectionStrategy = -201456

const DAQmxErrorNoCDAQSyncBetweenPhysAndSimulatedDevs = -201455

const DAQmxErrorUnableToContainCards = -201454

const DAQmxErrorFindDisconnectedBetweenPhysAndSimDeviceStatesInvalid = -201453

const DAQmxErrorOperationAborted = -201452

const DAQmxErrorTwoPortsRequired = -201451

const DAQmxErrorDeviceDoesNotSupportCDAQSyncConnections = -201450

const DAQmxErrorInvalidcDAQSyncPortConnectionFormat = -201449

const DAQmxErrorRosetteMeasurementsNotSpecified = -201448

const DAQmxErrorInvalidNumOfPhysChansForDeltaRosette = -201447

const DAQmxErrorInvalidNumOfPhysChansForTeeRosette = -201446

const DAQmxErrorRosetteStrainChanNamesNeeded = -201445

const DAQmxErrorMultideviceWithOnDemandTiming = -201444

const DAQmxErrorFREQOUTCannotProduceDesiredFrequency3 = -201443

const DAQmxErrorTwoEdgeSeparationSameTerminalSameEdge = -201442

const DAQmxErrorDontMixSyncPulseAndSampClkTimebaseOn449x = -201441

const DAQmxErrorNeitherRefClkNorSampClkTimebaseConfiguredForDSASync = -201440

const DAQmxErrorRetriggeringFiniteCONotAllowed = -201439

const DAQmxErrorDeviceRebootedFromWDTTimeout = -201438

const DAQmxErrorTimeoutValueExceedsMaximum = -201437

const DAQmxErrorSharingDifferentWireModes = -201436

const DAQmxErrorCantPrimeWithEmptyBuffer = -201435

const DAQmxErrorConfigFailedBecauseWatchdogExpired = -201434

const DAQmxErrorWriteFailedBecauseWatchdogChangedLineDirection = -201433

const DAQmxErrorMultipleSubsytemCalibration = -201432

const DAQmxErrorIncorrectChannelForOffsetAdjustment = -201431

const DAQmxErrorInvalidNumRefVoltagesToWrite = -201430

const DAQmxErrorStartTrigDelayWithDSAModule = -201429

const DAQmxErrorMoreThanOneSyncPulseDetected = -201428

const DAQmxErrorDevNotSupportedWithinDAQmxAPI = -201427

const DAQmxErrorDevsWithoutSyncStrategies = -201426

const DAQmxErrorDevsWithoutCommonSyncStrategy = -201425

const DAQmxErrorSyncStrategiesCannotSync = -201424

const DAQmxErrorChassisCommunicationInterrupted = -201423

const DAQmxErrorUnknownCardPowerProfileInCarrier = -201422

const DAQmxErrorAttrNotSupportedOnAccessory = -201421

const DAQmxErrorNetworkDeviceReservedByAnotherHost = -201420

const DAQmxErrorIncorrectFirmwareFileUploaded = -201419

const DAQmxErrorInvalidFirmwareFileUploaded = -201418

const DAQmxErrorInTimerTimeoutOnArm = -201417

const DAQmxErrorCantExceedSlotRelayDriveLimit = -201416

const DAQmxErrorModuleUnsupportedFor9163 = -201415

const DAQmxErrorConnectionsNotSupported = -201414

const DAQmxErrorAccessoryNotPresent = -201413

const DAQmxErrorSpecifiedAccessoryChannelsNotPresentOnDevice = -201412

const DAQmxErrorConnectionsNotSupportedOnAccessory = -201411

const DAQmxErrorRateTooFastForHWTSP = -201410

const DAQmxErrorDelayFromSampleClockOutOfRangeForHWTSP = -201409

const DAQmxErrorAveragingWhenNotInternalHWTSP = -201408

const DAQmxErrorAttributeNotSupportedUnlessHWTSP = -201407

const DAQmxErrorFiveVoltDetectFailed = -201406

const DAQmxErrorAnalogBusStateInconsistent = -201405

const DAQmxErrorCardDetectedDoesNotMatchExpectedCard = -201404

const DAQmxErrorLoggingStartNewFileNotCalled = -201403

const DAQmxErrorLoggingSampsPerFileNotDivisible = -201402

const DAQmxErrorRetrievingNetworkDeviceProperties = -201401

const DAQmxErrorFilePreallocationFailed = -201400

const DAQmxErrorModuleMismatchInSameTimedTask = -201399

const DAQmxErrorInvalidAttributeValuePossiblyDueToOtherAttributeValues = -201398

const DAQmxErrorChangeDetectionStoppedToPreventDeviceHang = -201397

const DAQmxErrorFilterDelayRemovalNotPosssibleWithAnalogTrigger = -201396

const DAQmxErrorNonbufferedOrNoChannels = -201395

const DAQmxErrorTristateLogicLevelNotSpecdForEntirePort = -201394

const DAQmxErrorTristateLogicLevelNotSupportedOnDigOutChan = -201393

const DAQmxErrorTristateLogicLevelNotSupported = -201392

const DAQmxErrorIncompleteGainAndCouplingCalAdjustment = -201391

const DAQmxErrorNetworkStatusConnectionLost = -201390

const DAQmxErrorModuleChangeDuringConnectionLoss = -201389

const DAQmxErrorNetworkDeviceNotReservedByHost = -201388

const DAQmxErrorDuplicateCalibrationAdjustmentInput = -201387

const DAQmxErrorSelfCalFailedContactTechSupport = -201386

const DAQmxErrorSelfCalFailedToConverge = -201385

const DAQmxErrorUnsupportedSimulatedModuleForSimulatedChassis = -201384

const DAQmxErrorLoggingWriteSizeTooBig = -201383

const DAQmxErrorLoggingWriteSizeNotDivisible = -201382

const DAQmxErrorMyDAQPowerRailFault = -201381

const DAQmxErrorDeviceDoesNotSupportThisOperation = -201380

const DAQmxErrorNetworkDevicesNotSupportedOnThisPlatform = -201379

const DAQmxErrorUnknownFirmwareVersion = -201378

const DAQmxErrorFirmwareIsUpdating = -201377

const DAQmxErrorAccessoryEEPROMIsCorrupt = -201376

const DAQmxErrorThrmcplLeadOffsetNullingCalNotSupported = -201375

const DAQmxErrorSelfCalFailedTryExtCal = -201374

const DAQmxErrorOutputP2PNotSupportedWithMultithreadedScripts = -201373

const DAQmxErrorThrmcplCalibrationChannelsOpen = -201372

const DAQmxErrorMDNSServiceInstanceAlreadyInUse = -201371

const DAQmxErrorIPAddressAlreadyInUse = -201370

const DAQmxErrorHostnameAlreadyInUse = -201369

const DAQmxErrorInvalidNumberOfCalAdjustmentPoints = -201368

const DAQmxErrorFilterOrDigitalSyncInternalSignal = -201367

const DAQmxErrorBadDDSSource = -201366

const DAQmxErrorOnboardRegenWithMoreThan16Channels = -201365

const DAQmxErrorTriggerTooFast = -201364

const DAQmxErrorMinMaxOutsideTableRange = -201363

const DAQmxErrorChannelExpansionWithInvalidAnalogTriggerDevice = -201362

const DAQmxErrorSyncPulseSrcInvalidForTask = -201361

const DAQmxErrorInvalidCarrierSlotNumberSpecd = -201360

const DAQmxErrorCardsMustBeInSameCarrier = -201359

const DAQmxErrorCardDevCarrierSimMustMatch = -201358

const DAQmxErrorDevMustHaveAtLeastOneCard = -201357

const DAQmxErrorCardTopologyError = -201356

const DAQmxErrorExceededCarrierPowerLimit = -201355

const DAQmxErrorCardsIncompatible = -201354

const DAQmxErrorAnalogBusNotValid = -201353

const DAQmxErrorReservationConflict = -201352

const DAQmxErrorMemMappedOnDemandNotSupported = -201351

const DAQmxErrorSlaveWithNoStartTriggerConfigured = -201350

const DAQmxErrorChannelExpansionWithDifferentTriggerDevices = -201349

const DAQmxErrorCounterSyncAndRetriggered = -201348

const DAQmxErrorNoExternalSyncPulseDetected = -201347

const DAQmxErrorSlaveAndNoExternalSyncPulse = -201346

const DAQmxErrorCustomTimingRequiredForAttribute = -201345

const DAQmxErrorCustomTimingModeNotSet = -201344

const DAQmxErrorAccessoryPowerTripped = -201343

const DAQmxErrorUnsupportedAccessory = -201342

const DAQmxErrorInvalidAccessoryChange = -201341

const DAQmxErrorFirmwareRequiresUpgrade = -201340

const DAQmxErrorFastExternalTimebaseNotSupportedForDevice = -201339

const DAQmxErrorInvalidShuntLocationForCalibration = -201338

const DAQmxErrorDeviceNameTooLong = -201337

const DAQmxErrorBridgeScalesUnsupported = -201336

const DAQmxErrorMismatchedElecPhysValues = -201335

const DAQmxErrorLinearRequiresUniquePoints = -201334

const DAQmxErrorMissingRequiredScalingParameter = -201333

const DAQmxErrorLoggingNotSupportOnOutputTasks = -201332

const DAQmxErrorMemoryMappedHardwareTimedNonBufferedUnsupported = -201331

const DAQmxErrorCannotUpdatePulseTrainWithAutoIncrementEnabled = -201330

const DAQmxErrorHWTimedSinglePointAndDataXferNotDMA = -201329

const DAQmxErrorSCCSecondStageEmpty = -201328

const DAQmxErrorSCCInvalidDualStageCombo = -201327

const DAQmxErrorSCCInvalidSecondStage = -201326

const DAQmxErrorSCCInvalidFirstStage = -201325

const DAQmxErrorCounterMultipleSampleClockedChannels = -201324

const DAQmxError2CounterMeasurementModeAndSampleClocked = -201323

const DAQmxErrorCantHaveBothMemMappedAndNonMemMappedTasks = -201322

const DAQmxErrorMemMappedDataReadByAnotherProcess = -201321

const DAQmxErrorRetriggeringInvalidForGivenSettings = -201320

const DAQmxErrorAIOverrun = -201319

const DAQmxErrorCOOverrun = -201318

const DAQmxErrorCounterMultipleBufferedChannels = -201317

const DAQmxErrorInvalidTimebaseForCOHWTSP = -201316

const DAQmxErrorWriteBeforeEvent = -201315

const DAQmxErrorCIOverrun = -201314

const DAQmxErrorCounterNonResponsiveAndReset = -201313

const DAQmxErrorMeasTypeOrChannelNotSupportedForLogging = -201312

const DAQmxErrorFileAlreadyOpenedForWrite = -201311

const DAQmxErrorTdmsNotFound = -201310

const DAQmxErrorGenericFileIO = -201309

const DAQmxErrorFiniteSTCCounterNotSupportedForLogging = -201308

const DAQmxErrorMeasurementTypeNotSupportedForLogging = -201307

const DAQmxErrorFileAlreadyOpened = -201306

const DAQmxErrorDiskFull = -201305

const DAQmxErrorFilePathInvalid = -201304

const DAQmxErrorFileVersionMismatch = -201303

const DAQmxErrorFileWriteProtected = -201302

const DAQmxErrorReadNotSupportedForLoggingMode = -201301

const DAQmxErrorAttributeNotSupportedWhenLogging = -201300

const DAQmxErrorLoggingModeNotSupportedNonBuffered = -201299

const DAQmxErrorPropertyNotSupportedWithConflictingProperty = -201298

const DAQmxErrorParallelSSHOnConnector1 = -201297

const DAQmxErrorCOOnlyImplicitSampleTimingTypeSupported = -201296

const DAQmxErrorCalibrationFailedAOOutOfRange = -201295

const DAQmxErrorCalibrationFailedAIOutOfRange = -201294

const DAQmxErrorCalPWMLinearityFailed = -201293

const DAQmxErrorOverrunUnderflowConfigurationCombo = -201292

const DAQmxErrorCannotWriteToFiniteCOTask = -201291

const DAQmxErrorNetworkDAQInvalidWEPKeyLength = -201290

const DAQmxErrorCalInputsShortedNotSupported = -201289

const DAQmxErrorCannotSetPropertyWhenTaskIsReserved = -201288

const DAQmxErrorMinus12VFuseBlown = -201287

const DAQmxErrorPlus12VFuseBlown = -201286

const DAQmxErrorPlus5VFuseBlown = -201285

const DAQmxErrorPlus3VFuseBlown = -201284

const DAQmxErrorDeviceSerialPortError = -201283

const DAQmxErrorPowerUpStateMachineNotDone = -201282

const DAQmxErrorTooManyTriggersSpecifiedInTask = -201281

const DAQmxErrorVerticalOffsetNotSupportedOnDevice = -201280

const DAQmxErrorInvalidCouplingForMeasurementType = -201279

const DAQmxErrorDigitalLineUpdateTooFastForDevice = -201278

const DAQmxErrorCertificateIsTooBigToTransfer = -201277

const DAQmxErrorOnlyPEMOrDERCertiticatesAccepted = -201276

const DAQmxErrorCalCouplingNotSupported = -201275

const DAQmxErrorDeviceNotSupportedIn64Bit = -201274

const DAQmxErrorNetworkDeviceInUse = -201273

const DAQmxErrorInvalidIPv4AddressFormat = -201272

const DAQmxErrorNetworkProductTypeMismatch = -201271

const DAQmxErrorOnlyPEMCertificatesAccepted = -201270

const DAQmxErrorCalibrationRequiresPrototypingBoardEnabled = -201269

const DAQmxErrorAllCurrentLimitingResourcesAlreadyTaken = -201268

const DAQmxErrorUserDefInfoStringBadLength = -201267

const DAQmxErrorPropertyNotFound = -201266

const DAQmxErrorOverVoltageProtectionActivated = -201265

const DAQmxErrorScaledIQWaveformTooLarge = -201264

const DAQmxErrorFirmwareFailedToDownload = -201263

const DAQmxErrorPropertyNotSupportedForBusType = -201262

const DAQmxErrorChangeRateWhileRunningCouldNotBeCompleted = -201261

const DAQmxErrorCannotQueryManualControlAttribute = -201260

const DAQmxErrorInvalidNetworkConfiguration = -201259

const DAQmxErrorInvalidWirelessConfiguration = -201258

const DAQmxErrorInvalidWirelessCountryCode = -201257

const DAQmxErrorInvalidWirelessChannel = -201256

const DAQmxErrorNetworkEEPROMHasChanged = -201255

const DAQmxErrorNetworkSerialNumberMismatch = -201254

const DAQmxErrorNetworkStatusDown = -201253

const DAQmxErrorNetworkTargetUnreachable = -201252

const DAQmxErrorNetworkTargetNotFound = -201251

const DAQmxErrorNetworkStatusTimedOut = -201250

const DAQmxErrorInvalidWirelessSecuritySelection = -201249

const DAQmxErrorNetworkDeviceConfigurationLocked = -201248

const DAQmxErrorNetworkDAQDeviceNotSupported = -201247

const DAQmxErrorNetworkDAQCannotCreateEmptySleeve = -201246

const DAQmxErrorUserDefInfoStringTooLong = -201245

const DAQmxErrorModuleTypeDoesNotMatchModuleTypeInDestination = -201244

const DAQmxErrorInvalidTEDSInterfaceAddress = -201243

const DAQmxErrorDevDoesNotSupportSCXIComm = -201242

const DAQmxErrorSCXICommDevConnector0MustBeCabledToModule = -201241

const DAQmxErrorSCXIModuleDoesNotSupportDigitizationMode = -201240

const DAQmxErrorDevDoesNotSupportMultiplexedSCXIDigitizationMode = -201239

const DAQmxErrorDevOrDevPhysChanDoesNotSupportSCXIDigitization = -201238

const DAQmxErrorInvalidPhysChanName = -201237

const DAQmxErrorSCXIChassisCommModeInvalid = -201236

const DAQmxErrorRequiredDependencyNotFound = -201235

const DAQmxErrorInvalidStorage = -201234

const DAQmxErrorInvalidObject = -201233

const DAQmxErrorStorageAlteredPriorToSave = -201232

const DAQmxErrorTaskDoesNotReferenceLocalChannel = -201231

const DAQmxErrorReferencedDevSimMustMatchTarget = -201230

const DAQmxErrorProgrammedIOFailsBecauseOfWatchdogTimer = -201229

const DAQmxErrorWatchdogTimerFailsBecauseOfProgrammedIO = -201228

const DAQmxErrorCantUseThisTimingEngineWithAPort = -201227

const DAQmxErrorProgrammedIOConflict = -201226

const DAQmxErrorChangeDetectionIncompatibleWithProgrammedIO = -201225

const DAQmxErrorTristateNotEnoughLines = -201224

const DAQmxErrorTristateConflict = -201223

const DAQmxErrorGenerateOrFiniteWaitExpectedBeforeBreakBlock = -201222

const DAQmxErrorBreakBlockNotAllowedInLoop = -201221

const DAQmxErrorClearTriggerNotAllowedInBreakBlock = -201220

const DAQmxErrorNestingNotAllowedInBreakBlock = -201219

const DAQmxErrorIfElseBlockNotAllowedInBreakBlock = -201218

const DAQmxErrorRepeatUntilTriggerLoopNotAllowedInBreakBlock = -201217

const DAQmxErrorWaitUntilTriggerNotAllowedInBreakBlock = -201216

const DAQmxErrorMarkerPosInvalidInBreakBlock = -201215

const DAQmxErrorInvalidWaitDurationInBreakBlock = -201214

const DAQmxErrorInvalidSubsetLengthInBreakBlock = -201213

const DAQmxErrorInvalidWaveformLengthInBreakBlock = -201212

const DAQmxErrorInvalidWaitDurationBeforeBreakBlock = -201211

const DAQmxErrorInvalidSubsetLengthBeforeBreakBlock = -201210

const DAQmxErrorInvalidWaveformLengthBeforeBreakBlock = -201209

const DAQmxErrorSampleRateTooHighForADCTimingMode = -201208

const DAQmxErrorActiveDevNotSupportedWithMultiDevTask = -201207

const DAQmxErrorRealDevAndSimDevNotSupportedInSameTask = -201206

const DAQmxErrorRTSISimMustMatchDevSim = -201205

const DAQmxErrorBridgeShuntCaNotSupported = -201204

const DAQmxErrorStrainShuntCaNotSupported = -201203

const DAQmxErrorGainTooLargeForGainCalConst = -201202

const DAQmxErrorOffsetTooLargeForOffsetCalConst = -201201

const DAQmxErrorElvisPrototypingBoardRemoved = -201200

const DAQmxErrorElvis2PowerRailFault = -201199

const DAQmxErrorElvis2PhysicalChansFault = -201198

const DAQmxErrorElvis2PhysicalChansThermalEvent = -201197

const DAQmxErrorRXBitErrorRateLimitExceeded = -201196

const DAQmxErrorPHYBitErrorRateLimitExceeded = -201195

const DAQmxErrorTwoPartAttributeCalledOutOfOrder = -201194

const DAQmxErrorInvalidSCXIChassisAddress = -201193

const DAQmxErrorCouldNotConnectToRemoteMXS = -201192

const DAQmxErrorExcitationStateRequiredForAttributes = -201191

const DAQmxErrorDeviceNotUsableUntilUSBReplug = -201190

const DAQmxErrorInputFIFOOverflowDuringCalibrationOnFullSpeedUSB = -201189

const DAQmxErrorInputFIFOOverflowDuringCalibration = -201188

const DAQmxErrorCJCChanConflictsWithNonThermocoupleChan = -201187

const DAQmxErrorCommDeviceForPXIBackplaneNotInRightmostSlot = -201186

const DAQmxErrorCommDeviceForPXIBackplaneNotInSameChassis = -201185

const DAQmxErrorCommDeviceForPXIBackplaneNotPXI = -201184

const DAQmxErrorInvalidCalExcitFrequency = -201183

const DAQmxErrorInvalidCalExcitVoltage = -201182

const DAQmxErrorInvalidAIInputSrc = -201181

const DAQmxErrorInvalidCalInputRef = -201180

const DAQmxErrordBReferenceValueNotGreaterThanZero = -201179

const DAQmxErrorSampleClockRateIsTooFastForSampleClockTiming = -201178

const DAQmxErrorDeviceNotUsableUntilColdStart = -201177

const DAQmxErrorSampleClockRateIsTooFastForBurstTiming = -201176

const DAQmxErrorDevImportFailedAssociatedResourceIDsNotSupported = -201175

const DAQmxErrorSCXI1600ImportNotSupported = -201174

const DAQmxErrorPowerSupplyConfigurationFailed = -201173

const DAQmxErrorIEPEWithDCNotAllowed = -201172

const DAQmxErrorMinTempForThermocoupleTypeOutsideAccuracyForPolyScaling = -201171

const DAQmxErrorDevImportFailedNoDeviceToOverwriteAndSimulationNotSupported = -201170

const DAQmxErrorDevImportFailedDeviceNotSupportedOnDestination = -201169

const DAQmxErrorFirmwareIsTooOld = -201168

const DAQmxErrorFirmwareCouldntUpdate = -201167

const DAQmxErrorFirmwareIsCorrupt = -201166

const DAQmxErrorFirmwareTooNew = -201165

const DAQmxErrorSampClockCannotBeExportedFromExternalSampClockSrc = -201164

const DAQmxErrorPhysChanReservedForInputWhenDesiredForOutput = -201163

const DAQmxErrorPhysChanReservedForOutputWhenDesiredForInput = -201162

const DAQmxErrorSpecifiedCDAQSlotNotEmpty = -201161

const DAQmxErrorDeviceDoesNotSupportSimulation = -201160

const DAQmxErrorInvalidCDAQSlotNumberSpecd = -201159

const DAQmxErrorCSeriesModSimMustMatchCDAQChassisSim = -201158

const DAQmxErrorSCCCabledDevMustNotBeSimWhenSCCCarrierIsNotSim = -201157

const DAQmxErrorSCCModSimMustMatchSCCCarrierSim = -201156

const DAQmxErrorSCXIModuleDoesNotSupportSimulation = -201155

const DAQmxErrorSCXICableDevMustNotBeSimWhenModIsNotSim = -201154

const DAQmxErrorSCXIDigitizerSimMustNotBeSimWhenModIsNotSim = -201153

const DAQmxErrorSCXIModSimMustMatchSCXIChassisSim = -201152

const DAQmxErrorSimPXIDevReqSlotAndChassisSpecd = -201151

const DAQmxErrorSimDevConflictWithRealDev = -201150

const DAQmxErrorInsufficientDataForCalibration = -201149

const DAQmxErrorTriggerChannelMustBeEnabled = -201148

const DAQmxErrorCalibrationDataConflictCouldNotBeResolved = -201147

const DAQmxErrorSoftwareTooNewForSelfCalibrationData = -201146

const DAQmxErrorSoftwareTooNewForExtCalibrationData = -201145

const DAQmxErrorSelfCalibrationDataTooNewForSoftware = -201144

const DAQmxErrorExtCalibrationDataTooNewForSoftware = -201143

const DAQmxErrorSoftwareTooNewForEEPROM = -201142

const DAQmxErrorEEPROMTooNewForSoftware = -201141

const DAQmxErrorSoftwareTooNewForHardware = -201140

const DAQmxErrorHardwareTooNewForSoftware = -201139

const DAQmxErrorTaskCannotRestartFirstSampNotAvailToGenerate = -201138

const DAQmxErrorOnlyUseStartTrigSrcPrptyWithDevDataLines = -201137

const DAQmxErrorOnlyUsePauseTrigSrcPrptyWithDevDataLines = -201136

const DAQmxErrorOnlyUseRefTrigSrcPrptyWithDevDataLines = -201135

const DAQmxErrorPauseTrigDigPatternSizeDoesNotMatchSrcSize = -201134

const DAQmxErrorLineConflictCDAQ = -201133

const DAQmxErrorCannotWriteBeyondFinalFiniteSample = -201132

const DAQmxErrorRefAndStartTriggerSrcCantBeSame = -201131

const DAQmxErrorMemMappingIncompatibleWithPhysChansInTask = -201130

const DAQmxErrorOutputDriveTypeMemMappingConflict = -201129

const DAQmxErrorCAPIDeviceIndexInvalid = -201128

const DAQmxErrorRatiometricDevicesMustUseExcitationForScaling = -201127

const DAQmxErrorPropertyRequiresPerDeviceCfg = -201126

const DAQmxErrorAICouplingAndAIInputSourceConflict = -201125

const DAQmxErrorOnlyOneTaskCanPerformDOMemoryMappingAtATime = -201124

const DAQmxErrorTooManyChansForAnalogRefTrigCDAQ = -201123

const DAQmxErrorSpecdPropertyValueIsIncompatibleWithSampleTimingType = -201122

const DAQmxErrorCPUNotSupportedRequireSSE = -201121

const DAQmxErrorSpecdPropertyValueIsIncompatibleWithSampleTimingResponseMode = -201120

const DAQmxErrorConflictingNextWriteIsLastAndRegenModeProperties = -201119

const DAQmxErrorMStudioOperationDoesNotSupportDeviceContext = -201118

const DAQmxErrorPropertyValueInChannelExpansionContextInvalid = -201117

const DAQmxErrorHWTimedNonBufferedAONotSupported = -201116

const DAQmxErrorWaveformLengthNotMultOfQuantum = -201115

const DAQmxErrorDSAExpansionMixedBoardsWrongOrderInPXIChassis = -201114

const DAQmxErrorPowerLevelTooLowForOOK = -201113

const DAQmxErrorDeviceComponentTestFailure = -201112

const DAQmxErrorUserDefinedWfmWithOOKUnsupported = -201111

const DAQmxErrorInvalidDigitalModulationUserDefinedWaveform = -201110

const DAQmxErrorBothRefInAndRefOutEnabled = -201109

const DAQmxErrorBothAnalogAndDigitalModulationEnabled = -201108

const DAQmxErrorBufferedOpsNotSupportedInSpecdSlotForCDAQ = -201107

const DAQmxErrorPhysChanNotSupportedInSpecdSlotForCDAQ = -201106

const DAQmxErrorResourceReservedWithConflictingSettings = -201105

const DAQmxErrorInconsistentAnalogTrigSettingsCDAQ = -201104

const DAQmxErrorTooManyChansForAnalogPauseTrigCDAQ = -201103

const DAQmxErrorAnalogTrigNotFirstInScanListCDAQ = -201102

const DAQmxErrorTooManyChansGivenTimingType = -201101

const DAQmxErrorSampClkTimebaseDivWithExtSampClk = -201100

const DAQmxErrorCantSaveTaskWithPerDeviceTimingProperties = -201099

const DAQmxErrorConflictingAutoZeroMode = -201098

const DAQmxErrorSampClkRateNotSupportedWithEAREnabled = -201097

const DAQmxErrorSampClkTimebaseRateNotSpecd = -201096

const DAQmxErrorSessionCorruptedByDLLReload = -201095

const DAQmxErrorActiveDevNotSupportedWithChanExpansion = -201094

const DAQmxErrorSampClkRateInvalid = -201093

const DAQmxErrorExtSyncPulseSrcCannotBeExported = -201092

const DAQmxErrorSyncPulseMinDelayToStartNeededForExtSyncPulseSrc = -201091

const DAQmxErrorSyncPulseSrcInvalid = -201090

const DAQmxErrorSampClkTimebaseRateInvalid = -201089

const DAQmxErrorSampClkTimebaseSrcInvalid = -201088

const DAQmxErrorSampClkRateMustBeSpecd = -201087

const DAQmxErrorInvalidAttributeName = -201086

const DAQmxErrorCJCChanNameMustBeSetWhenCJCSrcIsScannableChan = -201085

const DAQmxErrorHiddenChanMissingInChansPropertyInCfgFile = -201084

const DAQmxErrorChanNamesNotSpecdInCfgFile = -201083

const DAQmxErrorDuplicateHiddenChanNamesInCfgFile = -201082

const DAQmxErrorDuplicateChanNameInCfgFile = -201081

const DAQmxErrorInvalidSCCModuleForSlotSpecd = -201080

const DAQmxErrorInvalidSCCSlotNumberSpecd = -201079

const DAQmxErrorInvalidSectionIdentifier = -201078

const DAQmxErrorInvalidSectionName = -201077

const DAQmxErrorDAQmxVersionNotSupported = -201076

const DAQmxErrorSWObjectsFoundInFile = -201075

const DAQmxErrorHWObjectsFoundInFile = -201074

const DAQmxErrorLocalChannelSpecdWithNoParentTask = -201073

const DAQmxErrorTaskReferencesMissingLocalChannel = -201072

const DAQmxErrorTaskReferencesLocalChannelFromOtherTask = -201071

const DAQmxErrorTaskMissingChannelProperty = -201070

const DAQmxErrorInvalidLocalChanName = -201069

const DAQmxErrorInvalidEscapeCharacterInString = -201068

const DAQmxErrorInvalidTableIdentifier = -201067

const DAQmxErrorValueFoundInInvalidColumn = -201066

const DAQmxErrorMissingStartOfTable = -201065

const DAQmxErrorFileMissingRequiredDAQmxHeader = -201064

const DAQmxErrorDeviceIDDoesNotMatch = -201063

const DAQmxErrorBufferedOperationsNotSupportedOnSelectedLines = -201062

const DAQmxErrorPropertyConflictsWithScale = -201061

const DAQmxErrorInvalidINIFileSyntax = -201060

const DAQmxErrorDeviceInfoFailedPXIChassisNotIdentified = -201059

const DAQmxErrorInvalidHWProductNumber = -201058

const DAQmxErrorInvalidHWProductType = -201057

const DAQmxErrorInvalidNumericFormatSpecd = -201056

const DAQmxErrorDuplicatePropertyInObject = -201055

const DAQmxErrorInvalidEnumValueSpecd = -201054

const DAQmxErrorTEDSSensorPhysicalChannelConflict = -201053

const DAQmxErrorTooManyPhysicalChansForTEDSInterfaceSpecd = -201052

const DAQmxErrorIncapableTEDSInterfaceControllingDeviceSpecd = -201051

const DAQmxErrorSCCCarrierSpecdIsMissing = -201050

const DAQmxErrorIncapableSCCDigitizingDeviceSpecd = -201049

const DAQmxErrorAccessorySettingNotApplicable = -201048

const DAQmxErrorDeviceAndConnectorSpecdAlreadyOccupied = -201047

const DAQmxErrorIllegalAccessoryTypeForDeviceSpecd = -201046

const DAQmxErrorInvalidDeviceConnectorNumberSpecd = -201045

const DAQmxErrorInvalidAccessoryName = -201044

const DAQmxErrorMoreThanOneMatchForSpecdDevice = -201043

const DAQmxErrorNoMatchForSpecdDevice = -201042

const DAQmxErrorProductTypeAndProductNumberConflict = -201041

const DAQmxErrorExtraPropertyDetectedInSpecdObject = -201040

const DAQmxErrorRequiredPropertyMissing = -201039

const DAQmxErrorCantSetAuthorForLocalChan = -201038

const DAQmxErrorInvalidTimeValue = -201037

const DAQmxErrorInvalidTimeFormat = -201036

const DAQmxErrorDigDevChansSpecdInModeOtherThanParallel = -201035

const DAQmxErrorCascadeDigitizationModeNotSupported = -201034

const DAQmxErrorSpecdSlotAlreadyOccupied = -201033

const DAQmxErrorInvalidSCXISlotNumberSpecd = -201032

const DAQmxErrorAddressAlreadyInUse = -201031

const DAQmxErrorSpecdDeviceDoesNotSupportRTSI = -201030

const DAQmxErrorSpecdDeviceIsAlreadyOnRTSIBus = -201029

const DAQmxErrorIdentifierInUse = -201028

const DAQmxErrorWaitForNextSampleClockOrReadDetected3OrMoreMissedSampClks = -201027

const DAQmxErrorHWTimedAndDataXferPIO = -201026

const DAQmxErrorNonBufferedAndHWTimed = -201025

const DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriodPolled = -201024

const DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriod2 = -201023

const DAQmxErrorCOCannotKeepUpInHWTimedSinglePointPolled = -201022

const DAQmxErrorWriteRecoveryCannotKeepUpInHWTimedSinglePoint = -201021

const DAQmxErrorNoChangeDetectionOnSelectedLineForDevice = -201020

const DAQmxErrorSMIOPauseTriggersNotSupportedWithChannelExpansion = -201019

const DAQmxErrorClockMasterForExternalClockNotLongestPipeline = -201018

const DAQmxErrorUnsupportedUnicodeByteOrderMarker = -201017

const DAQmxErrorTooManyInstructionsInLoopInScript = -201016

const DAQmxErrorPLLNotLocked = -201015

const DAQmxErrorIfElseBlockNotAllowedInFiniteRepeatLoopInScript = -201014

const DAQmxErrorIfElseBlockNotAllowedInConditionalRepeatLoopInScript = -201013

const DAQmxErrorClearIsLastInstructionInIfElseBlockInScript = -201012

const DAQmxErrorInvalidWaitDurationBeforeIfElseBlockInScript = -201011

const DAQmxErrorMarkerPosInvalidBeforeIfElseBlockInScript = -201010

const DAQmxErrorInvalidSubsetLengthBeforeIfElseBlockInScript = -201009

const DAQmxErrorInvalidWaveformLengthBeforeIfElseBlockInScript = -201008

const DAQmxErrorGenerateOrFiniteWaitInstructionExpectedBeforeIfElseBlockInScript = -201007

const DAQmxErrorCalPasswordNotSupported = -201006

const DAQmxErrorSetupCalNeededBeforeAdjustCal = -201005

const DAQmxErrorMultipleChansNotSupportedDuringCalSetup = -201004

const DAQmxErrorDevCannotBeAccessed = -201003

const DAQmxErrorSampClkRateDoesntMatchSampClkSrc = -201002

const DAQmxErrorSampClkRateNotSupportedWithEARDisabled = -201001

const DAQmxErrorLabVIEWVersionDoesntSupportDAQmxEvents = -201000

const DAQmxErrorCOReadyForNewValNotSupportedWithOnDemand = -200999

const DAQmxErrorCIHWTimedSinglePointNotSupportedForMeasType = -200998

const DAQmxErrorOnDemandNotSupportedWithHWTimedSinglePoint = -200997

const DAQmxErrorHWTimedSinglePointAndDataXferNotProgIO = -200996

const DAQmxErrorMemMapAndHWTimedSinglePoint = -200995

const DAQmxErrorCannotSetPropertyWhenHWTimedSinglePointTaskIsRunning = -200994

const DAQmxErrorCTROutSampClkPeriodShorterThanGenPulseTrainPeriod = -200993

const DAQmxErrorTooManyEventsGenerated = -200992

const DAQmxErrorMStudioCppRemoveEventsBeforeStop = -200991

const DAQmxErrorCAPICannotRegisterSyncEventsFromMultipleThreads = -200990

const DAQmxErrorReadWaitNextSampClkWaitMismatchTwo = -200989

const DAQmxErrorReadWaitNextSampClkWaitMismatchOne = -200988

const DAQmxErrorDAQmxSignalEventTypeNotSupportedByChanTypesOrDevicesInTask = -200987

const DAQmxErrorCannotUnregisterDAQmxSoftwareEventWhileTaskIsRunning = -200986

const DAQmxErrorAutoStartWriteNotAllowedEventRegistered = -200985

const DAQmxErrorAutoStartReadNotAllowedEventRegistered = -200984

const DAQmxErrorCannotGetPropertyWhenTaskNotReservedCommittedOrRunning = -200983

const DAQmxErrorSignalEventsNotSupportedByDevice = -200982

const DAQmxErrorEveryNSamplesAcqIntoBufferEventNotSupportedByDevice = -200981

const DAQmxErrorEveryNSampsTransferredFromBufferEventNotSupportedByDevice = -200980

const DAQmxErrorCAPISyncEventsTaskStateChangeNotAllowedFromDifferentThread = -200979

const DAQmxErrorDAQmxSWEventsWithDifferentCallMechanisms = -200978

const DAQmxErrorCantSaveChanWithPolyCalScaleAndAllowInteractiveEdit = -200977

const DAQmxErrorChanDoesNotSupportCJC = -200976

const DAQmxErrorCOReadyForNewValNotSupportedWithHWTimedSinglePoint = -200975

const DAQmxErrorDACAllowConnToGndNotSupportedByDevWhenRefSrcExt = -200974

const DAQmxErrorCantGetPropertyTaskNotRunning = -200973

const DAQmxErrorCantSetPropertyTaskNotRunning = -200972

const DAQmxErrorCantSetPropertyTaskNotRunningCommitted = -200971

const DAQmxErrorAIEveryNSampsEventIntervalNotMultipleOf2 = -200970

const DAQmxErrorInvalidTEDSPhysChanNotAI = -200969

const DAQmxErrorCAPICannotPerformTaskOperationInAsyncCallback = -200968

const DAQmxErrorEveryNSampsTransferredFromBufferEventAlreadyRegistered = -200967

const DAQmxErrorEveryNSampsAcqIntoBufferEventAlreadyRegistered = -200966

const DAQmxErrorEveryNSampsTransferredFromBufferNotForInput = -200965

const DAQmxErrorEveryNSampsAcqIntoBufferNotForOutput = -200964

const DAQmxErrorAOSampTimingTypeDifferentIn2Tasks = -200963

const DAQmxErrorCouldNotDownloadFirmwareHWDamaged = -200962

const DAQmxErrorCouldNotDownloadFirmwareFileMissingOrDamaged = -200961

const DAQmxErrorCannotRegisterDAQmxSoftwareEventWhileTaskIsRunning = -200960

const DAQmxErrorDifferentRawDataCompression = -200959

const DAQmxErrorConfiguredTEDSInterfaceDevNotDetected = -200958

const DAQmxErrorCompressedSampSizeExceedsResolution = -200957

const DAQmxErrorChanDoesNotSupportCompression = -200956

const DAQmxErrorDifferentRawDataFormats = -200955

const DAQmxErrorSampClkOutputTermIncludesStartTrigSrc = -200954

const DAQmxErrorStartTrigSrcEqualToSampClkSrc = -200953

const DAQmxErrorEventOutputTermIncludesTrigSrc = -200952

const DAQmxErrorCOMultipleWritesBetweenSampClks = -200951

const DAQmxErrorDoneEventAlreadyRegistered = -200950

const DAQmxErrorSignalEventAlreadyRegistered = -200949

const DAQmxErrorCannotHaveTimedLoopAndDAQmxSignalEventsInSameTask = -200948

const DAQmxErrorNeedLabVIEW711PatchToUseDAQmxEvents = -200947

const DAQmxErrorStartFailedDueToWriteFailure = -200946

const DAQmxErrorDataXferCustomThresholdNotDMAXferMethodSpecifiedForDev = -200945

const DAQmxErrorDataXferRequestConditionNotSpecifiedForCustomThreshold = -200944

const DAQmxErrorDataXferCustomThresholdNotSpecified = -200943

const DAQmxErrorCAPISyncCallbackNotSupportedOnThisPlatform = -200942

const DAQmxErrorCalChanReversePolyCoefNotSpecd = -200941

const DAQmxErrorCalChanForwardPolyCoefNotSpecd = -200940

const DAQmxErrorChanCalRepeatedNumberInPreScaledVals = -200939

const DAQmxErrorChanCalTableNumScaledNotEqualNumPrescaledVals = -200938

const DAQmxErrorChanCalTableScaledValsNotSpecd = -200937

const DAQmxErrorChanCalTablePreScaledValsNotSpecd = -200936

const DAQmxErrorChanCalScaleTypeNotSet = -200935

const DAQmxErrorChanCalExpired = -200934

const DAQmxErrorChanCalExpirationDateNotSet = -200933

const DAQmxError3OutputPortCombinationGivenSampTimingType653x = -200932

const DAQmxError3InputPortCombinationGivenSampTimingType653x = -200931

const DAQmxError2OutputPortCombinationGivenSampTimingType653x = -200930

const DAQmxError2InputPortCombinationGivenSampTimingType653x = -200929

const DAQmxErrorPatternMatcherMayBeUsedByOneTrigOnly = -200928

const DAQmxErrorNoChansSpecdForPatternSource = -200927

const DAQmxErrorChangeDetectionChanNotInTask = -200926

const DAQmxErrorChangeDetectionChanNotTristated = -200925

const DAQmxErrorWaitModeValueNotSupportedNonBuffered = -200924

const DAQmxErrorWaitModePropertyNotSupportedNonBuffered = -200923

const DAQmxErrorCantSavePerLineConfigDigChanSoInteractiveEditsAllowed = -200922

const DAQmxErrorCantSaveNonPortMultiLineDigChanSoInteractiveEditsAllowed = -200921

const DAQmxErrorBufferSizeNotMultipleOfEveryNSampsEventIntervalNoIrqOnDev = -200920

const DAQmxErrorGlobalTaskNameAlreadyChanName = -200919

const DAQmxErrorGlobalChanNameAlreadyTaskName = -200918

const DAQmxErrorAOEveryNSampsEventIntervalNotMultipleOf2 = -200917

const DAQmxErrorSampleTimebaseDivisorNotSupportedGivenTimingType = -200916

const DAQmxErrorHandshakeEventOutputTermNotSupportedGivenTimingType = -200915

const DAQmxErrorChangeDetectionOutputTermNotSupportedGivenTimingType = -200914

const DAQmxErrorReadyForTransferOutputTermNotSupportedGivenTimingType = -200913

const DAQmxErrorRefTrigOutputTermNotSupportedGivenTimingType = -200912

const DAQmxErrorStartTrigOutputTermNotSupportedGivenTimingType = -200911

const DAQmxErrorSampClockOutputTermNotSupportedGivenTimingType = -200910

const DAQmxError20MhzTimebaseNotSupportedGivenTimingType = -200909

const DAQmxErrorSampClockSourceNotSupportedGivenTimingType = -200908

const DAQmxErrorRefTrigTypeNotSupportedGivenTimingType = -200907

const DAQmxErrorPauseTrigTypeNotSupportedGivenTimingType = -200906

const DAQmxErrorHandshakeTrigTypeNotSupportedGivenTimingType = -200905

const DAQmxErrorStartTrigTypeNotSupportedGivenTimingType = -200904

const DAQmxErrorRefClkSrcNotSupported = -200903

const DAQmxErrorDataVoltageLowAndHighIncompatible = -200902

const DAQmxErrorInvalidCharInDigPatternString = -200901

const DAQmxErrorCantUsePort3AloneGivenSampTimingTypeOn653x = -200900

const DAQmxErrorCantUsePort1AloneGivenSampTimingTypeOn653x = -200899

const DAQmxErrorPartialUseOfPhysicalLinesWithinPortNotSupported653x = -200898

const DAQmxErrorPhysicalChanNotSupportedGivenSampTimingType653x = -200897

const DAQmxErrorCanExportOnlyDigEdgeTrigs = -200896

const DAQmxErrorRefTrigDigPatternSizeDoesNotMatchSourceSize = -200895

const DAQmxErrorStartTrigDigPatternSizeDoesNotMatchSourceSize = -200894

const DAQmxErrorChangeDetectionRisingAndFallingEdgeChanDontMatch = -200893

const DAQmxErrorPhysicalChansForChangeDetectionAndPatternMatch653x = -200892

const DAQmxErrorCanExportOnlyOnboardSampClk = -200891

const DAQmxErrorInternalSampClkNotRisingEdge = -200890

const DAQmxErrorRefTrigDigPatternChanNotInTask = -200889

const DAQmxErrorRefTrigDigPatternChanNotTristated = -200888

const DAQmxErrorStartTrigDigPatternChanNotInTask = -200887

const DAQmxErrorStartTrigDigPatternChanNotTristated = -200886

const DAQmxErrorPXIStarAndClock10Sync = -200885

const DAQmxErrorGlobalChanCannotBeSavedSoInteractiveEditsAllowed = -200884

const DAQmxErrorTaskCannotBeSavedSoInteractiveEditsAllowed = -200883

const DAQmxErrorInvalidGlobalChan = -200882

const DAQmxErrorEveryNSampsEventAlreadyRegistered = -200881

const DAQmxErrorEveryNSampsEventIntervalZeroNotSupported = -200880

const DAQmxErrorChanSizeTooBigForU16PortWrite = -200879

const DAQmxErrorChanSizeTooBigForU16PortRead = -200878

const DAQmxErrorBufferSizeNotMultipleOfEveryNSampsEventIntervalWhenDMA = -200877

const DAQmxErrorWriteWhenTaskNotRunningCOTicks = -200876

const DAQmxErrorWriteWhenTaskNotRunningCOFreq = -200875

const DAQmxErrorWriteWhenTaskNotRunningCOTime = -200874

const DAQmxErrorAOMinMaxNotSupportedDACRangeTooSmall = -200873

const DAQmxErrorAOMinMaxNotSupportedGivenDACRange = -200872

const DAQmxErrorAOMinMaxNotSupportedGivenDACRangeAndOffsetVal = -200871

const DAQmxErrorAOMinMaxNotSupportedDACOffsetValInappropriate = -200870

const DAQmxErrorAOMinMaxNotSupportedGivenDACOffsetVal = -200869

const DAQmxErrorAOMinMaxNotSupportedDACRefValTooSmall = -200868

const DAQmxErrorAOMinMaxNotSupportedGivenDACRefVal = -200867

const DAQmxErrorAOMinMaxNotSupportedGivenDACRefAndOffsetVal = -200866

const DAQmxErrorWhenAcqCompAndNumSampsPerChanExceedsOnBrdBufSize = -200865

const DAQmxErrorWhenAcqCompAndNoRefTrig = -200864

const DAQmxErrorWaitForNextSampClkNotSupported = -200863

const DAQmxErrorDevInUnidentifiedPXIChassis = -200862

const DAQmxErrorMaxSoundPressureMicSensitivitRelatedAIPropertiesNotSupportedByDev = -200861

const DAQmxErrorMaxSoundPressureAndMicSensitivityNotSupportedByDev = -200860

const DAQmxErrorAOBufferSizeZeroForSampClkTimingType = -200859

const DAQmxErrorAOCallWriteBeforeStartForSampClkTimingType = -200858

const DAQmxErrorInvalidCalLowPassCutoffFreq = -200857

const DAQmxErrorSimulationCannotBeDisabledForDevCreatedAsSimulatedDev = -200856

const DAQmxErrorCannotAddNewDevsAfterTaskConfiguration = -200855

const DAQmxErrorDifftSyncPulseSrcAndSampClkTimebaseSrcDevMultiDevTask = -200854

const DAQmxErrorTermWithoutDevInMultiDevTask = -200853

const DAQmxErrorSyncNoDevSampClkTimebaseOrSyncPulseInPXISlot2 = -200852

const DAQmxErrorPhysicalChanNotOnThisConnector = -200851

const DAQmxErrorNumSampsToWaitNotGreaterThanZeroInScript = -200850

const DAQmxErrorNumSampsToWaitNotMultipleOfAlignmentQuantumInScript = -200849

const DAQmxErrorEveryNSamplesEventNotSupportedForNonBufferedTasks = -200848

const DAQmxErrorBufferedAndDataXferPIO = -200847

const DAQmxErrorCannotWriteWhenAutoStartFalseAndTaskNotRunning = -200846

const DAQmxErrorNonBufferedAndDataXferInterrupts = -200845

const DAQmxErrorWriteFailedMultipleCtrsWithFREQOUT = -200844

const DAQmxErrorReadNotCompleteBefore3SampClkEdges = -200843

const DAQmxErrorCtrHWTimedSinglePointAndDataXferNotProgIO = -200842

const DAQmxErrorPrescalerNot1ForInputTerminal = -200841

const DAQmxErrorPrescalerNot1ForTimebaseSrc = -200840

const DAQmxErrorSampClkTimingTypeWhenTristateIsFalse = -200839

const DAQmxErrorOutputBufferSizeNotMultOfXferSize = -200838

const DAQmxErrorSampPerChanNotMultOfXferSize = -200837

const DAQmxErrorWriteToTEDSFailed = -200836

const DAQmxErrorSCXIDevNotUsablePowerTurnedOff = -200835

const DAQmxErrorCannotReadWhenAutoStartFalseBufSizeZeroAndTaskNotRunning = -200834

const DAQmxErrorCannotReadWhenAutoStartFalseHWTimedSinglePtAndTaskNotRunning = -200833

const DAQmxErrorCannotReadWhenAutoStartFalseOnDemandAndTaskNotRunning = -200832

const DAQmxErrorSimultaneousAOWhenNotOnDemandTiming = -200831

const DAQmxErrorMemMapAndSimultaneousAO = -200830

const DAQmxErrorWriteFailedMultipleCOOutputTypes = -200829

const DAQmxErrorWriteToTEDSNotSupportedOnRT = -200828

const DAQmxErrorVirtualTEDSDataFileError = -200827

const DAQmxErrorTEDSSensorDataError = -200826

const DAQmxErrorDataSizeMoreThanSizeOfEEPROMOnTEDS = -200825

const DAQmxErrorPROMOnTEDSContainsBasicTEDSData = -200824

const DAQmxErrorPROMOnTEDSAlreadyWritten = -200823

const DAQmxErrorTEDSDoesNotContainPROM = -200822

const DAQmxErrorHWTimedSinglePointNotSupportedAI = -200821

const DAQmxErrorHWTimedSinglePointOddNumChansInAITask = -200820

const DAQmxErrorCantUseOnlyOnBoardMemWithProgrammedIO = -200819

const DAQmxErrorSwitchDevShutDownDueToHighTemp = -200818

const DAQmxErrorExcitationNotSupportedWhenTermCfgDiff = -200817

const DAQmxErrorTEDSMinElecValGEMaxElecVal = -200816

const DAQmxErrorTEDSMinPhysValGEMaxPhysVal = -200815

const DAQmxErrorCIOnboardClockNotSupportedAsInputTerm = -200814

const DAQmxErrorInvalidSampModeForPositionMeas = -200813

const DAQmxErrorTrigWhenAOHWTimedSinglePtSampMode = -200812

const DAQmxErrorDAQmxCantUseStringDueToUnknownChar = -200811

const DAQmxErrorDAQmxCantRetrieveStringDueToUnknownChar = -200810

const DAQmxErrorClearTEDSNotSupportedOnRT = -200809

const DAQmxErrorCfgTEDSNotSupportedOnRT = -200808

const DAQmxErrorProgFilterClkCfgdToDifferentMinPulseWidthBySameTask1PerDev = -200807

const DAQmxErrorProgFilterClkCfgdToDifferentMinPulseWidthByAnotherTask1PerDev = -200806

const DAQmxErrorNoLastExtCalDateTimeLastExtCalNotDAQmx = -200804

const DAQmxErrorCannotWriteNotStartedAutoStartFalseNotOnDemandHWTimedSglPt = -200803

const DAQmxErrorCannotWriteNotStartedAutoStartFalseNotOnDemandBufSizeZero = -200802

const DAQmxErrorCOInvalidTimingSrcDueToSignal = -200801

const DAQmxErrorCIInvalidTimingSrcForSampClkDueToSampTimingType = -200800

const DAQmxErrorCIInvalidTimingSrcForEventCntDueToSampMode = -200799

const DAQmxErrorNoChangeDetectOnNonInputDigLineForDev = -200798

const DAQmxErrorEmptyStringTermNameNotSupported = -200797

const DAQmxErrorMemMapEnabledForHWTimedNonBufferedAO = -200796

const DAQmxErrorDevOnboardMemOverflowDuringHWTimedNonBufferedGen = -200795

const DAQmxErrorCODAQmxWriteMultipleChans = -200794

const DAQmxErrorCantMaintainExistingValueAOSync = -200793

const DAQmxErrorMStudioMultiplePhysChansNotSupported = -200792

const DAQmxErrorCantConfigureTEDSForChan = -200791

const DAQmxErrorWriteDataTypeTooSmall = -200790

const DAQmxErrorReadDataTypeTooSmall = -200789

const DAQmxErrorMeasuredBridgeOffsetTooHigh = -200788

const DAQmxErrorStartTrigConflictWithCOHWTimedSinglePt = -200787

const DAQmxErrorSampClkRateExtSampClkTimebaseRateMismatch = -200786

const DAQmxErrorInvalidTimingSrcDueToSampTimingType = -200785

const DAQmxErrorVirtualTEDSFileNotFound = -200784

const DAQmxErrorMStudioNoForwardPolyScaleCoeffs = -200783

const DAQmxErrorMStudioNoReversePolyScaleCoeffs = -200782

const DAQmxErrorMStudioNoPolyScaleCoeffsUseCalc = -200781

const DAQmxErrorMStudioNoForwardPolyScaleCoeffsUseCalc = -200780

const DAQmxErrorMStudioNoReversePolyScaleCoeffsUseCalc = -200779

const DAQmxErrorCOSampModeSampTimingTypeSampClkConflict = -200778

const DAQmxErrorDevCannotProduceMinPulseWidth = -200777

const DAQmxErrorCannotProduceMinPulseWidthGivenPropertyValues = -200776

const DAQmxErrorTermCfgdToDifferentMinPulseWidthByAnotherTask = -200775

const DAQmxErrorTermCfgdToDifferentMinPulseWidthByAnotherProperty = -200774

const DAQmxErrorDigSyncNotAvailableOnTerm = -200773

const DAQmxErrorDigFilterNotAvailableOnTerm = -200772

const DAQmxErrorDigFilterEnabledMinPulseWidthNotCfg = -200771

const DAQmxErrorDigFilterAndSyncBothEnabled = -200770

const DAQmxErrorHWTimedSinglePointAOAndDataXferNotProgIO = -200769

const DAQmxErrorNonBufferedAOAndDataXferNotProgIO = -200768

const DAQmxErrorProgIODataXferForBufferedAO = -200767

const DAQmxErrorTEDSLegacyTemplateIDInvalidOrUnsupported = -200766

const DAQmxErrorTEDSMappingMethodInvalidOrUnsupported = -200765

const DAQmxErrorTEDSLinearMappingSlopeZero = -200764

const DAQmxErrorAIInputBufferSizeNotMultOfXferSize = -200763

const DAQmxErrorNoSyncPulseExtSampClkTimebase = -200762

const DAQmxErrorNoSyncPulseAnotherTaskRunning = -200761

const DAQmxErrorAOMinMaxNotInGainRange = -200760

const DAQmxErrorAOMinMaxNotInDACRange = -200759

const DAQmxErrorDevOnlySupportsSampClkTimingAO = -200758

const DAQmxErrorDevOnlySupportsSampClkTimingAI = -200757

const DAQmxErrorTEDSIncompatibleSensorAndMeasType = -200756

const DAQmxErrorTEDSMultipleCalTemplatesNotSupported = -200755

const DAQmxErrorTEDSTemplateParametersNotSupported = -200754

const DAQmxErrorParsingTEDSData = -200753

const DAQmxErrorMultipleActivePhysChansNotSupported = -200752

const DAQmxErrorNoChansSpecdForChangeDetect = -200751

const DAQmxErrorInvalidCalVoltageForGivenGain = -200750

const DAQmxErrorInvalidCalGain = -200749

const DAQmxErrorMultipleWritesBetweenSampClks = -200748

const DAQmxErrorInvalidAcqTypeForFREQOUT = -200747

const DAQmxErrorSuitableTimebaseNotFoundTimeCombo2 = -200746

const DAQmxErrorSuitableTimebaseNotFoundFrequencyCombo2 = -200745

const DAQmxErrorRefClkRateRefClkSrcMismatch = -200744

const DAQmxErrorNoTEDSTerminalBlock = -200743

const DAQmxErrorCorruptedTEDSMemory = -200742

const DAQmxErrorTEDSNotSupported = -200741

const DAQmxErrorTimingSrcTaskStartedBeforeTimedLoop = -200740

const DAQmxErrorPropertyNotSupportedForTimingSrc = -200739

const DAQmxErrorTimingSrcDoesNotExist = -200738

const DAQmxErrorInputBufferSizeNotEqualSampsPerChanForFiniteSampMode = -200737

const DAQmxErrorFREQOUTCannotProduceDesiredFrequency2 = -200736

const DAQmxErrorExtRefClkRateNotSpecified = -200735

const DAQmxErrorDeviceDoesNotSupportDMADataXferForNonBufferedAcq = -200734

const DAQmxErrorDigFilterMinPulseWidthSetWhenTristateIsFalse = -200733

const DAQmxErrorDigFilterEnableSetWhenTristateIsFalse = -200732

const DAQmxErrorNoHWTimingWithOnDemand = -200731

const DAQmxErrorCannotDetectChangesWhenTristateIsFalse = -200730

const DAQmxErrorCannotHandshakeWhenTristateIsFalse = -200729

const DAQmxErrorLinesUsedForStaticInputNotForHandshakingControl = -200728

const DAQmxErrorLinesUsedForHandshakingControlNotForStaticInput = -200727

const DAQmxErrorLinesUsedForStaticInputNotForHandshakingInput = -200726

const DAQmxErrorLinesUsedForHandshakingInputNotForStaticInput = -200725

const DAQmxErrorDifferentDITristateValsForChansInTask = -200724

const DAQmxErrorTimebaseCalFreqVarianceTooLarge = -200723

const DAQmxErrorTimebaseCalFailedToConverge = -200722

const DAQmxErrorInadequateResolutionForTimebaseCal = -200721

const DAQmxErrorInvalidAOGainCalConst = -200720

const DAQmxErrorInvalidAOOffsetCalConst = -200719

const DAQmxErrorInvalidAIGainCalConst = -200718

const DAQmxErrorInvalidAIOffsetCalConst = -200717

const DAQmxErrorDigOutputOverrun = -200716

const DAQmxErrorDigInputOverrun = -200715

const DAQmxErrorAcqStoppedDriverCantXferDataFastEnough = -200714

const DAQmxErrorChansCantAppearInSameTask = -200713

const DAQmxErrorInputCfgFailedBecauseWatchdogExpired = -200712

const DAQmxErrorAnalogTrigChanNotExternal = -200711

const DAQmxErrorTooManyChansForInternalAIInputSrc = -200710

const DAQmxErrorTEDSSensorNotDetected = -200709

const DAQmxErrorPrptyGetSpecdActiveItemFailedDueToDifftValues = -200708

const DAQmxErrorRoutingDestTermPXIClk10InNotInSlot2 = -200706

const DAQmxErrorRoutingDestTermPXIStarXNotInSlot2 = -200705

const DAQmxErrorRoutingSrcTermPXIStarXNotInSlot2 = -200704

const DAQmxErrorRoutingSrcTermPXIStarInSlot16AndAbove = -200703

const DAQmxErrorRoutingDestTermPXIStarInSlot16AndAbove = -200702

const DAQmxErrorRoutingDestTermPXIStarInSlot2 = -200701

const DAQmxErrorRoutingSrcTermPXIStarInSlot2 = -200700

const DAQmxErrorRoutingDestTermPXIChassisNotIdentified = -200699

const DAQmxErrorRoutingSrcTermPXIChassisNotIdentified = -200698

const DAQmxErrorFailedToAcquireCalData = -200697

const DAQmxErrorBridgeOffsetNullingCalNotSupported = -200696

const DAQmxErrorAIMaxNotSpecified = -200695

const DAQmxErrorAIMinNotSpecified = -200694

const DAQmxErrorOddTotalBufferSizeToWrite = -200693

const DAQmxErrorOddTotalNumSampsToWrite = -200692

const DAQmxErrorBufferWithWaitMode = -200691

const DAQmxErrorBufferWithHWTimedSinglePointSampMode = -200690

const DAQmxErrorCOWritePulseLowTicksNotSupported = -200689

const DAQmxErrorCOWritePulseHighTicksNotSupported = -200688

const DAQmxErrorCOWritePulseLowTimeOutOfRange = -200687

const DAQmxErrorCOWritePulseHighTimeOutOfRange = -200686

const DAQmxErrorCOWriteFreqOutOfRange = -200685

const DAQmxErrorCOWriteDutyCycleOutOfRange = -200684

const DAQmxErrorInvalidInstallation = -200683

const DAQmxErrorRefTrigMasterSessionUnavailable = -200682

const DAQmxErrorRouteFailedBecauseWatchdogExpired = -200681

const DAQmxErrorDeviceShutDownDueToHighTemp = -200680

const DAQmxErrorNoMemMapWhenHWTimedSinglePoint = -200679

const DAQmxErrorWriteFailedBecauseWatchdogExpired = -200678

const DAQmxErrorDifftInternalAIInputSrcs = -200677

const DAQmxErrorDifftAIInputSrcInOneChanGroup = -200676

const DAQmxErrorInternalAIInputSrcInMultChanGroups = -200675

const DAQmxErrorSwitchOpFailedDueToPrevError = -200674

const DAQmxErrorWroteMultiSampsUsingSingleSampWrite = -200673

const DAQmxErrorMismatchedInputArraySizes = -200672

const DAQmxErrorCantExceedRelayDriveLimit = -200671

const DAQmxErrorDACRngLowNotEqualToMinusRefVal = -200670

const DAQmxErrorCantAllowConnectDACToGnd = -200669

const DAQmxErrorWatchdogTimeoutOutOfRangeAndNotSpecialVal = -200668

const DAQmxErrorNoWatchdogOutputOnPortReservedForInput = -200667

const DAQmxErrorNoInputOnPortCfgdForWatchdogOutput = -200666

const DAQmxErrorWatchdogExpirationStateNotEqualForLinesInPort = -200665

const DAQmxErrorCannotPerformOpWhenTaskNotReserved = -200664

const DAQmxErrorPowerupStateNotSupported = -200663

const DAQmxErrorWatchdogTimerNotSupported = -200662

const DAQmxErrorOpNotSupportedWhenRefClkSrcNone = -200661

const DAQmxErrorSampClkRateUnavailable = -200660

const DAQmxErrorPrptyGetSpecdSingleActiveChanFailedDueToDifftVals = -200659

const DAQmxErrorPrptyGetImpliedActiveChanFailedDueToDifftVals = -200658

const DAQmxErrorPrptyGetSpecdActiveChanFailedDueToDifftVals = -200657

const DAQmxErrorNoRegenWhenUsingBrdMem = -200656

const DAQmxErrorNonbufferedReadMoreThanSampsPerChan = -200655

const DAQmxErrorWatchdogExpirationTristateNotSpecdForEntirePort = -200654

const DAQmxErrorPowerupTristateNotSpecdForEntirePort = -200653

const DAQmxErrorPowerupStateNotSpecdForEntirePort = -200652

const DAQmxErrorCantSetWatchdogExpirationOnDigInChan = -200651

const DAQmxErrorCantSetPowerupStateOnDigInChan = -200650

const DAQmxErrorPhysChanNotInTask = -200649

const DAQmxErrorPhysChanDevNotInTask = -200648

const DAQmxErrorDigInputNotSupported = -200647

const DAQmxErrorDigFilterIntervalNotEqualForLines = -200646

const DAQmxErrorDigFilterIntervalAlreadyCfgd = -200645

const DAQmxErrorCantResetExpiredWatchdog = -200644

const DAQmxErrorActiveChanTooManyLinesSpecdWhenGettingPrpty = -200643

const DAQmxErrorActiveChanNotSpecdWhenGetting1LinePrpty = -200642

const DAQmxErrorDigPrptyCannotBeSetPerLine = -200641

const DAQmxErrorSendAdvCmpltAfterWaitForTrigInScanlist = -200640

const DAQmxErrorDisconnectionRequiredInScanlist = -200639

const DAQmxErrorTwoWaitForTrigsAfterConnectionInScanlist = -200638

const DAQmxErrorActionSeparatorRequiredAfterBreakingConnectionInScanlist = -200637

const DAQmxErrorConnectionInScanlistMustWaitForTrig = -200636

const DAQmxErrorActionNotSupportedTaskNotWatchdog = -200635

const DAQmxErrorWfmNameSameAsScriptName = -200634

const DAQmxErrorScriptNameSameAsWfmName = -200633

const DAQmxErrorDSFStopClock = -200632

const DAQmxErrorDSFReadyForStartClock = -200631

const DAQmxErrorWriteOffsetNotMultOfIncr = -200630

const DAQmxErrorDifferentPrptyValsNotSupportedOnDev = -200629

const DAQmxErrorRefAndPauseTrigConfigured = -200628

const DAQmxErrorFailedToEnableHighSpeedInputClock = -200627

const DAQmxErrorEmptyPhysChanInPowerUpStatesArray = -200626

const DAQmxErrorActivePhysChanTooManyLinesSpecdWhenGettingPrpty = -200625

const DAQmxErrorActivePhysChanNotSpecdWhenGetting1LinePrpty = -200624

const DAQmxErrorPXIDevTempCausedShutDown = -200623

const DAQmxErrorInvalidNumSampsToWrite = -200622

const DAQmxErrorOutputFIFOUnderflow2 = -200621

const DAQmxErrorRepeatedAIPhysicalChan = -200620

const DAQmxErrorMultScanOpsInOneChassis = -200619

const DAQmxErrorInvalidAIChanOrder = -200618

const DAQmxErrorReversePowerProtectionActivated = -200617

const DAQmxErrorInvalidAsynOpHandle = -200616

const DAQmxErrorFailedToEnableHighSpeedOutput = -200615

const DAQmxErrorCannotReadPastEndOfRecord = -200614

const DAQmxErrorAcqStoppedToPreventInputBufferOverwriteOneDataXferMech = -200613

const DAQmxErrorZeroBasedChanIndexInvalid = -200612

const DAQmxErrorNoChansOfGivenTypeInTask = -200611

const DAQmxErrorSampClkSrcInvalidForOutputValidForInput = -200610

const DAQmxErrorOutputBufSizeTooSmallToStartGen = -200609

const DAQmxErrorInputBufSizeTooSmallToStartAcq = -200608

const DAQmxErrorExportTwoSignalsOnSameTerminal = -200607

const DAQmxErrorChanIndexInvalid = -200606

const DAQmxErrorRangeSyntaxNumberTooBig = -200605

const DAQmxErrorNULLPtr = -200604

const DAQmxErrorScaledMinEqualMax = -200603

const DAQmxErrorPreScaledMinEqualMax = -200602

const DAQmxErrorPropertyNotSupportedForScaleType = -200601

const DAQmxErrorChannelNameGenerationNumberTooBig = -200600

const DAQmxErrorRepeatedNumberInScaledValues = -200599

const DAQmxErrorRepeatedNumberInPreScaledValues = -200598

const DAQmxErrorLinesAlreadyReservedForOutput = -200597

const DAQmxErrorSwitchOperationChansSpanMultipleDevsInList = -200596

const DAQmxErrorInvalidIDInListAtBeginningOfSwitchOperation = -200595

const DAQmxErrorMStudioInvalidPolyDirection = -200594

const DAQmxErrorMStudioPropertyGetWhileTaskNotVerified = -200593

const DAQmxErrorRangeWithTooManyObjects = -200592

const DAQmxErrorCppDotNetAPINegativeBufferSize = -200591

const DAQmxErrorCppCantRemoveInvalidEventHandler = -200590

const DAQmxErrorCppCantRemoveEventHandlerTwice = -200589

const DAQmxErrorCppCantRemoveOtherObjectsEventHandler = -200588

const DAQmxErrorDigLinesReservedOrUnavailable = -200587

const DAQmxErrorDSFFailedToResetStream = -200586

const DAQmxErrorDSFReadyForOutputNotAsserted = -200585

const DAQmxErrorSampToWritePerChanNotMultipleOfIncr = -200584

const DAQmxErrorAOPropertiesCauseVoltageBelowMin = -200583

const DAQmxErrorAOPropertiesCauseVoltageOverMax = -200582

const DAQmxErrorPropertyNotSupportedWhenRefClkSrcNone = -200581

const DAQmxErrorAIMaxTooSmall = -200580

const DAQmxErrorAIMaxTooLarge = -200579

const DAQmxErrorAIMinTooSmall = -200578

const DAQmxErrorAIMinTooLarge = -200577

const DAQmxErrorBuiltInCJCSrcNotSupported = -200576

const DAQmxErrorTooManyPostTrigSampsPerChan = -200575

const DAQmxErrorTrigLineNotFoundSingleDevRoute = -200574

const DAQmxErrorDifferentInternalAIInputSources = -200573

const DAQmxErrorDifferentAIInputSrcInOneChanGroup = -200572

const DAQmxErrorInternalAIInputSrcInMultipleChanGroups = -200571

const DAQmxErrorCAPIChanIndexInvalid = -200570

const DAQmxErrorCollectionDoesNotMatchChanType = -200569

const DAQmxErrorOutputCantStartChangedRegenerationMode = -200568

const DAQmxErrorOutputCantStartChangedBufferSize = -200567

const DAQmxErrorChanSizeTooBigForU32PortWrite = -200566

const DAQmxErrorChanSizeTooBigForU8PortWrite = -200565

const DAQmxErrorChanSizeTooBigForU32PortRead = -200564

const DAQmxErrorChanSizeTooBigForU8PortRead = -200563

const DAQmxErrorInvalidDigDataWrite = -200562

const DAQmxErrorInvalidAODataWrite = -200561

const DAQmxErrorWaitUntilDoneDoesNotIndicateDone = -200560

const DAQmxErrorMultiChanTypesInTask = -200559

const DAQmxErrorMultiDevsInTask = -200558

const DAQmxErrorCannotSetPropertyWhenTaskRunning = -200557

const DAQmxErrorCannotGetPropertyWhenTaskNotCommittedOrRunning = -200556

const DAQmxErrorLeadingUnderscoreInString = -200555

const DAQmxErrorTrailingSpaceInString = -200554

const DAQmxErrorLeadingSpaceInString = -200553

const DAQmxErrorInvalidCharInString = -200552

const DAQmxErrorDLLBecameUnlocked = -200551

const DAQmxErrorDLLLock = -200550

const DAQmxErrorSelfCalConstsInvalid = -200549

const DAQmxErrorInvalidTrigCouplingExceptForExtTrigChan = -200548

const DAQmxErrorWriteFailsBufferSizeAutoConfigured = -200547

const DAQmxErrorExtCalAdjustExtRefVoltageFailed = -200546

const DAQmxErrorSelfCalFailedExtNoiseOrRefVoltageOutOfCal = -200545

const DAQmxErrorExtCalTemperatureNotDAQmx = -200544

const DAQmxErrorExtCalDateTimeNotDAQmx = -200543

const DAQmxErrorSelfCalTemperatureNotDAQmx = -200542

const DAQmxErrorSelfCalDateTimeNotDAQmx = -200541

const DAQmxErrorDACRefValNotSet = -200540

const DAQmxErrorAnalogMultiSampWriteNotSupported = -200539

const DAQmxErrorInvalidActionInControlTask = -200538

const DAQmxErrorPolyCoeffsInconsistent = -200537

const DAQmxErrorSensorValTooLow = -200536

const DAQmxErrorSensorValTooHigh = -200535

const DAQmxErrorWaveformNameTooLong = -200534

const DAQmxErrorIdentifierTooLongInScript = -200533

const DAQmxErrorUnexpectedIDFollowingSwitchChanName = -200532

const DAQmxErrorRelayNameNotSpecifiedInList = -200531

const DAQmxErrorUnexpectedIDFollowingRelayNameInList = -200530

const DAQmxErrorUnexpectedIDFollowingSwitchOpInList = -200529

const DAQmxErrorInvalidLineGrouping = -200528

const DAQmxErrorCtrMinMax = -200527

const DAQmxErrorWriteChanTypeMismatch = -200526

const DAQmxErrorReadChanTypeMismatch = -200525

const DAQmxErrorWriteNumChansMismatch = -200524

const DAQmxErrorOneChanReadForMultiChanTask = -200523

const DAQmxErrorCannotSelfCalDuringExtCal = -200522

const DAQmxErrorMeasCalAdjustOscillatorPhaseDAC = -200521

const DAQmxErrorInvalidCalConstCalADCAdjustment = -200520

const DAQmxErrorInvalidCalConstOscillatorFreqDACValue = -200519

const DAQmxErrorInvalidCalConstOscillatorPhaseDACValue = -200518

const DAQmxErrorInvalidCalConstOffsetDACValue = -200517

const DAQmxErrorInvalidCalConstGainDACValue = -200516

const DAQmxErrorInvalidNumCalADCReadsToAverage = -200515

const DAQmxErrorInvalidCfgCalAdjustDirectPathOutputImpedance = -200514

const DAQmxErrorInvalidCfgCalAdjustMainPathOutputImpedance = -200513

const DAQmxErrorInvalidCfgCalAdjustMainPathPostAmpGainAndOffset = -200512

const DAQmxErrorInvalidCfgCalAdjustMainPathPreAmpGain = -200511

const DAQmxErrorInvalidCfgCalAdjustMainPreAmpOffset = -200510

const DAQmxErrorMeasCalAdjustCalADC = -200509

const DAQmxErrorMeasCalAdjustOscillatorFrequency = -200508

const DAQmxErrorMeasCalAdjustDirectPathOutputImpedance = -200507

const DAQmxErrorMeasCalAdjustMainPathOutputImpedance = -200506

const DAQmxErrorMeasCalAdjustDirectPathGain = -200505

const DAQmxErrorMeasCalAdjustMainPathPostAmpGainAndOffset = -200504

const DAQmxErrorMeasCalAdjustMainPathPreAmpGain = -200503

const DAQmxErrorMeasCalAdjustMainPathPreAmpOffset = -200502

const DAQmxErrorInvalidDateTimeInEEPROM = -200501

const DAQmxErrorUnableToLocateErrorResources = -200500

const DAQmxErrorDotNetAPINotUnsigned32BitNumber = -200499

const DAQmxErrorInvalidRangeOfObjectsSyntaxInString = -200498

const DAQmxErrorAttemptToEnableLineNotPreviouslyDisabled = -200497

const DAQmxErrorInvalidCharInPattern = -200496

const DAQmxErrorIntermediateBufferFull = -200495

const DAQmxErrorLoadTaskFailsBecauseNoTimingOnDev = -200494

const DAQmxErrorCAPIReservedParamNotNULLNorEmpty = -200493

const DAQmxErrorCAPIReservedParamNotNULL = -200492

const DAQmxErrorCAPIReservedParamNotZero = -200491

const DAQmxErrorSampleValueOutOfRange = -200490

const DAQmxErrorChanAlreadyInTask = -200489

const DAQmxErrorVirtualChanDoesNotExist = -200488

const DAQmxErrorChanNotInTask = -200486

const DAQmxErrorTaskNotInDataNeighborhood = -200485

const DAQmxErrorCantSaveTaskWithoutReplace = -200484

const DAQmxErrorCantSaveChanWithoutReplace = -200483

const DAQmxErrorDevNotInTask = -200482

const DAQmxErrorDevAlreadyInTask = -200481

const DAQmxErrorCanNotPerformOpWhileTaskRunning = -200479

const DAQmxErrorCanNotPerformOpWhenNoChansInTask = -200478

const DAQmxErrorCanNotPerformOpWhenNoDevInTask = -200477

const DAQmxErrorCannotPerformOpWhenTaskNotRunning = -200475

const DAQmxErrorOperationTimedOut = -200474

const DAQmxErrorCannotReadWhenAutoStartFalseAndTaskNotRunningOrCommitted = -200473

const DAQmxErrorCannotWriteWhenAutoStartFalseAndTaskNotRunningOrCommitted = -200472

const DAQmxErrorTaskVersionNew = -200470

const DAQmxErrorChanVersionNew = -200469

const DAQmxErrorEmptyString = -200467

const DAQmxErrorChannelSizeTooBigForPortReadType = -200466

const DAQmxErrorChannelSizeTooBigForPortWriteType = -200465

const DAQmxErrorExpectedNumberOfChannelsVerificationFailed = -200464

const DAQmxErrorNumLinesMismatchInReadOrWrite = -200463

const DAQmxErrorOutputBufferEmpty = -200462

const DAQmxErrorInvalidChanName = -200461

const DAQmxErrorReadNoInputChansInTask = -200460

const DAQmxErrorWriteNoOutputChansInTask = -200459

const DAQmxErrorPropertyNotSupportedNotInputTask = -200457

const DAQmxErrorPropertyNotSupportedNotOutputTask = -200456

const DAQmxErrorGetPropertyNotInputBufferedTask = -200455

const DAQmxErrorGetPropertyNotOutputBufferedTask = -200454

const DAQmxErrorInvalidTimeoutVal = -200453

const DAQmxErrorAttributeNotSupportedInTaskContext = -200452

const DAQmxErrorAttributeNotQueryableUnlessTaskIsCommitted = -200451

const DAQmxErrorAttributeNotSettableWhenTaskIsRunning = -200450

const DAQmxErrorDACRngLowNotMinusRefValNorZero = -200449

const DAQmxErrorDACRngHighNotEqualRefVal = -200448

const DAQmxErrorUnitsNotFromCustomScale = -200447

const DAQmxErrorInvalidVoltageReadingDuringExtCal = -200446

const DAQmxErrorCalFunctionNotSupported = -200445

const DAQmxErrorInvalidPhysicalChanForCal = -200444

const DAQmxErrorExtCalNotComplete = -200443

const DAQmxErrorCantSyncToExtStimulusFreqDuringCal = -200442

const DAQmxErrorUnableToDetectExtStimulusFreqDuringCal = -200441

const DAQmxErrorInvalidCloseAction = -200440

const DAQmxErrorExtCalFunctionOutsideExtCalSession = -200439

const DAQmxErrorInvalidCalArea = -200438

const DAQmxErrorExtCalConstsInvalid = -200437

const DAQmxErrorStartTrigDelayWithExtSampClk = -200436

const DAQmxErrorDelayFromSampClkWithExtConv = -200435

const DAQmxErrorFewerThan2PreScaledVals = -200434

const DAQmxErrorFewerThan2ScaledValues = -200433

const DAQmxErrorPhysChanOutputType = -200432

const DAQmxErrorPhysChanMeasType = -200431

const DAQmxErrorInvalidPhysChanType = -200430

const DAQmxErrorLabVIEWEmptyTaskOrChans = -200429

const DAQmxErrorLabVIEWInvalidTaskOrChans = -200428

const DAQmxErrorInvalidRefClkRate = -200427

const DAQmxErrorInvalidExtTrigImpedance = -200426

const DAQmxErrorHystTrigLevelAIMax = -200425

const DAQmxErrorLineNumIncompatibleWithVideoSignalFormat = -200424

const DAQmxErrorTrigWindowAIMinAIMaxCombo = -200423

const DAQmxErrorTrigAIMinAIMax = -200422

const DAQmxErrorHystTrigLevelAIMin = -200421

const DAQmxErrorInvalidSampRateConsiderRIS = -200420

const DAQmxErrorInvalidReadPosDuringRIS = -200419

const DAQmxErrorImmedTrigDuringRISMode = -200418

const DAQmxErrorTDCNotEnabledDuringRISMode = -200417

const DAQmxErrorMultiRecWithRIS = -200416

const DAQmxErrorInvalidRefClkSrc = -200415

const DAQmxErrorInvalidSampClkSrc = -200414

const DAQmxErrorInsufficientOnBoardMemForNumRecsAndSamps = -200413

const DAQmxErrorInvalidAIAttenuation = -200412

const DAQmxErrorACCouplingNotAllowedWith50OhmImpedance = -200411

const DAQmxErrorInvalidRecordNum = -200410

const DAQmxErrorZeroSlopeLinearScale = -200409

const DAQmxErrorZeroReversePolyScaleCoeffs = -200408

const DAQmxErrorZeroForwardPolyScaleCoeffs = -200407

const DAQmxErrorNoReversePolyScaleCoeffs = -200406

const DAQmxErrorNoForwardPolyScaleCoeffs = -200405

const DAQmxErrorNoPolyScaleCoeffs = -200404

const DAQmxErrorReversePolyOrderLessThanNumPtsToCompute = -200403

const DAQmxErrorReversePolyOrderNotPositive = -200402

const DAQmxErrorNumPtsToComputeNotPositive = -200401

const DAQmxErrorWaveformLengthNotMultipleOfIncr = -200400

const DAQmxErrorCAPINoExtendedErrorInfoAvailable = -200399

const DAQmxErrorCVIFunctionNotFoundInDAQmxDLL = -200398

const DAQmxErrorCVIFailedToLoadDAQmxDLL = -200397

const DAQmxErrorNoCommonTrigLineForImmedRoute = -200396

const DAQmxErrorNoCommonTrigLineForTaskRoute = -200395

const DAQmxErrorF64PrptyValNotUnsignedInt = -200394

const DAQmxErrorRegisterNotWritable = -200393

const DAQmxErrorInvalidOutputVoltageAtSampClkRate = -200392

const DAQmxErrorStrobePhaseShiftDCMBecameUnlocked = -200391

const DAQmxErrorDrivePhaseShiftDCMBecameUnlocked = -200390

const DAQmxErrorClkOutPhaseShiftDCMBecameUnlocked = -200389

const DAQmxErrorOutputBoardClkDCMBecameUnlocked = -200388

const DAQmxErrorInputBoardClkDCMBecameUnlocked = -200387

const DAQmxErrorInternalClkDCMBecameUnlocked = -200386

const DAQmxErrorDCMLock = -200385

const DAQmxErrorDataLineReservedForDynamicOutput = -200384

const DAQmxErrorInvalidRefClkSrcGivenSampClkSrc = -200383

const DAQmxErrorNoPatternMatcherAvailable = -200382

const DAQmxErrorInvalidDelaySampRateBelowPhaseShiftDCMThresh = -200381

const DAQmxErrorStrainGageCalibration = -200380

const DAQmxErrorInvalidExtClockFreqAndDivCombo = -200379

const DAQmxErrorCustomScaleDoesNotExist = -200378

const DAQmxErrorOnlyFrontEndChanOpsDuringScan = -200377

const DAQmxErrorInvalidOptionForDigitalPortChannel = -200376

const DAQmxErrorUnsupportedSignalTypeExportSignal = -200375

const DAQmxErrorInvalidSignalTypeExportSignal = -200374

const DAQmxErrorUnsupportedTrigTypeSendsSWTrig = -200373

const DAQmxErrorInvalidTrigTypeSendsSWTrig = -200372

const DAQmxErrorRepeatedPhysicalChan = -200371

const DAQmxErrorResourcesInUseForRouteInTask = -200370

const DAQmxErrorResourcesInUseForRoute = -200369

const DAQmxErrorRouteNotSupportedByHW = -200368

const DAQmxErrorResourcesInUseForExportSignalPolarity = -200367

const DAQmxErrorResourcesInUseForInversionInTask = -200366

const DAQmxErrorResourcesInUseForInversion = -200365

const DAQmxErrorExportSignalPolarityNotSupportedByHW = -200364

const DAQmxErrorInversionNotSupportedByHW = -200363

const DAQmxErrorOverloadedChansExistNotRead = -200362

const DAQmxErrorInputFIFOOverflow2 = -200361

const DAQmxErrorCJCChanNotSpecd = -200360

const DAQmxErrorCtrExportSignalNotPossible = -200359

const DAQmxErrorRefTrigWhenContinuous = -200358

const DAQmxErrorIncompatibleSensorOutputAndDeviceInputRanges = -200357

const DAQmxErrorCustomScaleNameUsed = -200356

const DAQmxErrorPropertyValNotSupportedByHW = -200355

const DAQmxErrorPropertyValNotValidTermName = -200354

const DAQmxErrorResourcesInUseForProperty = -200353

const DAQmxErrorCJCChanAlreadyUsed = -200352

const DAQmxErrorForwardPolynomialCoefNotSpecd = -200351

const DAQmxErrorTableScaleNumPreScaledAndScaledValsNotEqual = -200350

const DAQmxErrorTableScalePreScaledValsNotSpecd = -200349

const DAQmxErrorTableScaleScaledValsNotSpecd = -200348

const DAQmxErrorIntermediateBufferSizeNotMultipleOfIncr = -200347

const DAQmxErrorEventPulseWidthOutOfRange = -200346

const DAQmxErrorEventDelayOutOfRange = -200345

const DAQmxErrorSampPerChanNotMultipleOfIncr = -200344

const DAQmxErrorCannotCalculateNumSampsTaskNotStarted = -200343

const DAQmxErrorScriptNotInMem = -200342

const DAQmxErrorOnboardMemTooSmall = -200341

const DAQmxErrorReadAllAvailableDataWithoutBuffer = -200340

const DAQmxErrorPulseActiveAtStart = -200339

const DAQmxErrorCalTempNotSupported = -200338

const DAQmxErrorDelayFromSampClkTooLong = -200337

const DAQmxErrorDelayFromSampClkTooShort = -200336

const DAQmxErrorAIConvRateTooHigh = -200335

const DAQmxErrorDelayFromStartTrigTooLong = -200334

const DAQmxErrorDelayFromStartTrigTooShort = -200333

const DAQmxErrorSampRateTooHigh = -200332

const DAQmxErrorSampRateTooLow = -200331

const DAQmxErrorPFI0UsedForAnalogAndDigitalSrc = -200330

const DAQmxErrorPrimingCfgFIFO = -200329

const DAQmxErrorCannotOpenTopologyCfgFile = -200328

const DAQmxErrorInvalidDTInsideWfmDataType = -200327

const DAQmxErrorRouteSrcAndDestSame = -200326

const DAQmxErrorReversePolynomialCoefNotSpecd = -200325

const DAQmxErrorDevAbsentOrUnavailable = -200324

const DAQmxErrorNoAdvTrigForMultiDevScan = -200323

const DAQmxErrorInterruptsInsufficientDataXferMech = -200322

const DAQmxErrorInvalidAttentuationBasedOnMinMax = -200321

const DAQmxErrorCabledModuleCannotRouteSSH = -200320

const DAQmxErrorCabledModuleCannotRouteConvClk = -200319

const DAQmxErrorInvalidExcitValForScaling = -200318

const DAQmxErrorNoDevMemForScript = -200317

const DAQmxErrorScriptDataUnderflow = -200316

const DAQmxErrorNoDevMemForWaveform = -200315

const DAQmxErrorStreamDCMBecameUnlocked = -200314

const DAQmxErrorStreamDCMLock = -200313

const DAQmxErrorWaveformNotInMem = -200312

const DAQmxErrorWaveformWriteOutOfBounds = -200311

const DAQmxErrorWaveformPreviouslyAllocated = -200310

const DAQmxErrorSampClkTbMasterTbDivNotAppropriateForSampTbSrc = -200309

const DAQmxErrorSampTbRateSampTbSrcMismatch = -200308

const DAQmxErrorMasterTbRateMasterTbSrcMismatch = -200307

const DAQmxErrorSampsPerChanTooBig = -200306

const DAQmxErrorFinitePulseTrainNotPossible = -200305

const DAQmxErrorExtMasterTimebaseRateNotSpecified = -200304

const DAQmxErrorExtSampClkSrcNotSpecified = -200303

const DAQmxErrorInputSignalSlowerThanMeasTime = -200302

const DAQmxErrorCannotUpdatePulseGenProperty = -200301

const DAQmxErrorInvalidTimingType = -200300

const DAQmxErrorPropertyUnavailWhenUsingOnboardMemory = -200297

const DAQmxErrorCannotWriteAfterStartWithOnboardMemory = -200295

const DAQmxErrorNotEnoughSampsWrittenForInitialXferRqstCondition = -200294

const DAQmxErrorNoMoreSpace = -200293

const DAQmxErrorSamplesCanNotYetBeWritten = -200292

const DAQmxErrorGenStoppedToPreventIntermediateBufferRegenOfOldSamples = -200291

const DAQmxErrorGenStoppedToPreventRegenOfOldSamples = -200290

const DAQmxErrorSamplesNoLongerWriteable = -200289

const DAQmxErrorSamplesWillNeverBeGenerated = -200288

const DAQmxErrorNegativeWriteSampleNumber = -200287

const DAQmxErrorNoAcqStarted = -200286

const DAQmxErrorSamplesNotYetAvailable = -200284

const DAQmxErrorAcqStoppedToPreventIntermediateBufferOverflow = -200283

const DAQmxErrorNoRefTrigConfigured = -200282

const DAQmxErrorCannotReadRelativeToRefTrigUntilDone = -200281

const DAQmxErrorSamplesNoLongerAvailable = -200279

const DAQmxErrorSamplesWillNeverBeAvailable = -200278

const DAQmxErrorNegativeReadSampleNumber = -200277

const DAQmxErrorExternalSampClkAndRefClkThruSameTerm = -200276

const DAQmxErrorExtSampClkRateTooLowForClkIn = -200275

const DAQmxErrorExtSampClkRateTooHighForBackplane = -200274

const DAQmxErrorSampClkRateAndDivCombo = -200273

const DAQmxErrorSampClkRateTooLowForDivDown = -200272

const DAQmxErrorProductOfAOMinAndGainTooSmall = -200271

const DAQmxErrorInterpolationRateNotPossible = -200270

const DAQmxErrorOffsetTooLarge = -200269

const DAQmxErrorOffsetTooSmall = -200268

const DAQmxErrorProductOfAOMaxAndGainTooLarge = -200267

const DAQmxErrorMinAndMaxNotSymmetric = -200266

const DAQmxErrorInvalidAnalogTrigSrc = -200265

const DAQmxErrorTooManyChansForAnalogRefTrig = -200264

const DAQmxErrorTooManyChansForAnalogPauseTrig = -200263

const DAQmxErrorTrigWhenOnDemandSampTiming = -200262

const DAQmxErrorInconsistentAnalogTrigSettings = -200261

const DAQmxErrorMemMapDataXferModeSampTimingCombo = -200260

const DAQmxErrorInvalidJumperedAttr = -200259

const DAQmxErrorInvalidGainBasedOnMinMax = -200258

const DAQmxErrorInconsistentExcit = -200257

const DAQmxErrorTopologyNotSupportedByCfgTermBlock = -200256

const DAQmxErrorBuiltInTempSensorNotSupported = -200255

const DAQmxErrorInvalidTerm = -200254

const DAQmxErrorCannotTristateTerm = -200253

const DAQmxErrorCannotTristateBusyTerm = -200252

const DAQmxErrorNoDMAChansAvailable = -200251

const DAQmxErrorInvalidWaveformLengthWithinLoopInScript = -200250

const DAQmxErrorInvalidSubsetLengthWithinLoopInScript = -200249

const DAQmxErrorMarkerPosInvalidForLoopInScript = -200248

const DAQmxErrorIntegerExpectedInScript = -200247

const DAQmxErrorPLLBecameUnlocked = -200246

const DAQmxErrorPLLLock = -200245

const DAQmxErrorDDCClkOutDCMBecameUnlocked = -200244

const DAQmxErrorDDCClkOutDCMLock = -200243

const DAQmxErrorClkDoublerDCMBecameUnlocked = -200242

const DAQmxErrorClkDoublerDCMLock = -200241

const DAQmxErrorSampClkDCMBecameUnlocked = -200240

const DAQmxErrorSampClkDCMLock = -200239

const DAQmxErrorSampClkTimebaseDCMBecameUnlocked = -200238

const DAQmxErrorSampClkTimebaseDCMLock = -200237

const DAQmxErrorAttrCannotBeReset = -200236

const DAQmxErrorExplanationNotFound = -200235

const DAQmxErrorWriteBufferTooSmall = -200234

const DAQmxErrorSpecifiedAttrNotValid = -200233

const DAQmxErrorAttrCannotBeRead = -200232

const DAQmxErrorAttrCannotBeSet = -200231

const DAQmxErrorNULLPtrForC_Api = -200230

const DAQmxErrorReadBufferTooSmall = -200229

const DAQmxErrorBufferTooSmallForString = -200228

const DAQmxErrorNoAvailTrigLinesOnDevice = -200227

const DAQmxErrorTrigBusLineNotAvail = -200226

const DAQmxErrorCouldNotReserveRequestedTrigLine = -200225

const DAQmxErrorTrigLineNotFound = -200224

const DAQmxErrorSCXI1126ThreshHystCombination = -200223

const DAQmxErrorAcqStoppedToPreventInputBufferOverwrite = -200222

const DAQmxErrorTimeoutExceeded = -200221

const DAQmxErrorInvalidDeviceID = -200220

const DAQmxErrorInvalidAOChanOrder = -200219

const DAQmxErrorSampleTimingTypeAndDataXferMode = -200218

const DAQmxErrorBufferWithOnDemandSampTiming = -200217

const DAQmxErrorBufferAndDataXferMode = -200216

const DAQmxErrorMemMapAndBuffer = -200215

const DAQmxErrorNoAnalogTrigHW = -200214

const DAQmxErrorTooManyPretrigPlusMinPostTrigSamps = -200213

const DAQmxErrorInconsistentUnitsSpecified = -200212

const DAQmxErrorMultipleRelaysForSingleRelayOp = -200211

const DAQmxErrorMultipleDevIDsPerChassisSpecifiedInList = -200210

const DAQmxErrorDuplicateDevIDInList = -200209

const DAQmxErrorInvalidRangeStatementCharInList = -200208

const DAQmxErrorInvalidDeviceIDInList = -200207

const DAQmxErrorTriggerPolarityConflict = -200206

const DAQmxErrorCannotScanWithCurrentTopology = -200205

const DAQmxErrorUnexpectedIdentifierInFullySpecifiedPathInList = -200204

const DAQmxErrorSwitchCannotDriveMultipleTrigLines = -200203

const DAQmxErrorInvalidRelayName = -200202

const DAQmxErrorSwitchScanlistTooBig = -200201

const DAQmxErrorSwitchChanInUse = -200200

const DAQmxErrorSwitchNotResetBeforeScan = -200199

const DAQmxErrorInvalidTopology = -200198

const DAQmxErrorAttrNotSupported = -200197

const DAQmxErrorUnexpectedEndOfActionsInList = -200196

const DAQmxErrorPowerLimitExceeded = -200195

const DAQmxErrorHWUnexpectedlyPoweredOffAndOn = -200194

const DAQmxErrorSwitchOperationNotSupported = -200193

const DAQmxErrorOnlyContinuousScanSupported = -200192

const DAQmxErrorSwitchDifferentTopologyWhenScanning = -200191

const DAQmxErrorDisconnectPathNotSameAsExistingPath = -200190

const DAQmxErrorConnectionNotPermittedOnChanReservedForRouting = -200189

const DAQmxErrorCannotConnectSrcChans = -200188

const DAQmxErrorCannotConnectChannelToItself = -200187

const DAQmxErrorChannelNotReservedForRouting = -200186

const DAQmxErrorCannotConnectChansDirectly = -200185

const DAQmxErrorChansAlreadyConnected = -200184

const DAQmxErrorChanDuplicatedInPath = -200183

const DAQmxErrorNoPathToDisconnect = -200182

const DAQmxErrorInvalidSwitchChan = -200181

const DAQmxErrorNoPathAvailableBetween2SwitchChans = -200180

const DAQmxErrorExplicitConnectionExists = -200179

const DAQmxErrorSwitchDifferentSettlingTimeWhenScanning = -200178

const DAQmxErrorOperationOnlyPermittedWhileScanning = -200177

const DAQmxErrorOperationNotPermittedWhileScanning = -200176

const DAQmxErrorHardwareNotResponding = -200175

const DAQmxErrorInvalidSampAndMasterTimebaseRateCombo = -200173

const DAQmxErrorNonZeroBufferSizeInProgIOXfer = -200172

const DAQmxErrorVirtualChanNameUsed = -200171

const DAQmxErrorPhysicalChanDoesNotExist = -200170

const DAQmxErrorMemMapOnlyForProgIOXfer = -200169

const DAQmxErrorTooManyChans = -200168

const DAQmxErrorCannotHaveCJTempWithOtherChans = -200167

const DAQmxErrorOutputBufferUnderwrite = -200166

const DAQmxErrorSensorInvalidCompletionResistance = -200163

const DAQmxErrorVoltageExcitIncompatibleWith2WireCfg = -200162

const DAQmxErrorIntExcitSrcNotAvailable = -200161

const DAQmxErrorCannotCreateChannelAfterTaskVerified = -200160

const DAQmxErrorLinesReservedForSCXIControl = -200159

const DAQmxErrorCouldNotReserveLinesForSCXIControl = -200158

const DAQmxErrorCalibrationFailed = -200157

const DAQmxErrorReferenceFrequencyInvalid = -200156

const DAQmxErrorReferenceResistanceInvalid = -200155

const DAQmxErrorReferenceCurrentInvalid = -200154

const DAQmxErrorReferenceVoltageInvalid = -200153

const DAQmxErrorEEPROMDataInvalid = -200152

const DAQmxErrorCabledModuleNotCapableOfRoutingAI = -200151

const DAQmxErrorChannelNotAvailableInParallelMode = -200150

const DAQmxErrorExternalTimebaseRateNotKnownForDelay = -200149

const DAQmxErrorFREQOUTCannotProduceDesiredFrequency = -200148

const DAQmxErrorMultipleCounterInputTask = -200147

const DAQmxErrorCounterStartPauseTriggerConflict = -200146

const DAQmxErrorCounterInputPauseTriggerAndSampleClockInvalid = -200145

const DAQmxErrorCounterOutputPauseTriggerInvalid = -200144

const DAQmxErrorCounterTimebaseRateNotSpecified = -200143

const DAQmxErrorCounterTimebaseRateNotFound = -200142

const DAQmxErrorCounterOverflow = -200141

const DAQmxErrorCounterNoTimebaseEdgesBetweenGates = -200140

const DAQmxErrorCounterMaxMinRangeFreq = -200139

const DAQmxErrorCounterMaxMinRangeTime = -200138

const DAQmxErrorSuitableTimebaseNotFoundTimeCombo = -200137

const DAQmxErrorSuitableTimebaseNotFoundFrequencyCombo = -200136

const DAQmxErrorInternalTimebaseSourceDivisorCombo = -200135

const DAQmxErrorInternalTimebaseSourceRateCombo = -200134

const DAQmxErrorInternalTimebaseRateDivisorSourceCombo = -200133

const DAQmxErrorExternalTimebaseRateNotknownForRate = -200132

const DAQmxErrorAnalogTrigChanNotFirstInScanList = -200131

const DAQmxErrorNoDivisorForExternalSignal = -200130

const DAQmxErrorAttributeInconsistentAcrossRepeatedPhysicalChannels = -200128

const DAQmxErrorCannotHandshakeWithPort0 = -200127

const DAQmxErrorControlLineConflictOnPortC = -200126

const DAQmxErrorLines4To7ConfiguredForOutput = -200125

const DAQmxErrorLines4To7ConfiguredForInput = -200124

const DAQmxErrorLines0To3ConfiguredForOutput = -200123

const DAQmxErrorLines0To3ConfiguredForInput = -200122

const DAQmxErrorPortConfiguredForOutput = -200121

const DAQmxErrorPortConfiguredForInput = -200120

const DAQmxErrorPortConfiguredForStaticDigitalOps = -200119

const DAQmxErrorPortReservedForHandshaking = -200118

const DAQmxErrorPortDoesNotSupportHandshakingDataIO = -200117

const DAQmxErrorCannotTristate8255OutputLines = -200116

const DAQmxErrorTemperatureOutOfRangeForCalibration = -200113

const DAQmxErrorCalibrationHandleInvalid = -200112

const DAQmxErrorPasswordRequired = -200111

const DAQmxErrorIncorrectPassword = -200110

const DAQmxErrorPasswordTooLong = -200109

const DAQmxErrorCalibrationSessionAlreadyOpen = -200108

const DAQmxErrorSCXIModuleIncorrect = -200107

const DAQmxErrorAttributeInconsistentAcrossChannelsOnDevice = -200106

const DAQmxErrorSCXI1122ResistanceChanNotSupportedForCfg = -200105

const DAQmxErrorBracketPairingMismatchInList = -200104

const DAQmxErrorInconsistentNumSamplesToWrite = -200103

const DAQmxErrorIncorrectDigitalPattern = -200102

const DAQmxErrorIncorrectNumChannelsToWrite = -200101

const DAQmxErrorIncorrectReadFunction = -200100

const DAQmxErrorPhysicalChannelNotSpecified = -200099

const DAQmxErrorMoreThanOneTerminal = -200098

const DAQmxErrorMoreThanOneActiveChannelSpecified = -200097

const DAQmxErrorInvalidNumberSamplesToRead = -200096

const DAQmxErrorAnalogWaveformExpected = -200095

const DAQmxErrorDigitalWaveformExpected = -200094

const DAQmxErrorActiveChannelNotSpecified = -200093

const DAQmxErrorFunctionNotSupportedForDeviceTasks = -200092

const DAQmxErrorFunctionNotInLibrary = -200091

const DAQmxErrorLibraryNotPresent = -200090

const DAQmxErrorDuplicateTask = -200089

const DAQmxErrorInvalidTask = -200088

const DAQmxErrorInvalidChannel = -200087

const DAQmxErrorInvalidSyntaxForPhysicalChannelRange = -200086

const DAQmxErrorMinNotLessThanMax = -200082

const DAQmxErrorSampleRateNumChansConvertPeriodCombo = -200081

const DAQmxErrorAODuringCounter1DMAConflict = -200079

const DAQmxErrorAIDuringCounter0DMAConflict = -200078

const DAQmxErrorInvalidAttributeValue = -200077

const DAQmxErrorSuppliedCurrentDataOutsideSpecifiedRange = -200076

const DAQmxErrorSuppliedVoltageDataOutsideSpecifiedRange = -200075

const DAQmxErrorCannotStoreCalConst = -200074

const DAQmxErrorSCXIModuleNotFound = -200073

const DAQmxErrorDuplicatePhysicalChansNotSupported = -200072

const DAQmxErrorTooManyPhysicalChansInList = -200071

const DAQmxErrorInvalidAdvanceEventTriggerType = -200070

const DAQmxErrorDeviceIsNotAValidSwitch = -200069

const DAQmxErrorDeviceDoesNotSupportScanning = -200068

const DAQmxErrorScanListCannotBeTimed = -200067

const DAQmxErrorConnectOperatorInvalidAtPointInList = -200066

const DAQmxErrorUnexpectedSwitchActionInList = -200065

const DAQmxErrorUnexpectedSeparatorInList = -200064

const DAQmxErrorExpectedTerminatorInList = -200063

const DAQmxErrorExpectedConnectOperatorInList = -200062

const DAQmxErrorExpectedSeparatorInList = -200061

const DAQmxErrorFullySpecifiedPathInListContainsRange = -200060

const DAQmxErrorConnectionSeparatorAtEndOfList = -200059

const DAQmxErrorIdentifierInListTooLong = -200058

const DAQmxErrorDuplicateDeviceIDInListWhenSettling = -200057

const DAQmxErrorChannelNameNotSpecifiedInList = -200056

const DAQmxErrorDeviceIDNotSpecifiedInList = -200055

const DAQmxErrorSemicolonDoesNotFollowRangeInList = -200054

const DAQmxErrorSwitchActionInListSpansMultipleDevices = -200053

const DAQmxErrorRangeWithoutAConnectActionInList = -200052

const DAQmxErrorInvalidIdentifierFollowingSeparatorInList = -200051

const DAQmxErrorInvalidChannelNameInList = -200050

const DAQmxErrorInvalidNumberInRepeatStatementInList = -200049

const DAQmxErrorInvalidTriggerLineInList = -200048

const DAQmxErrorInvalidIdentifierInListFollowingDeviceID = -200047

const DAQmxErrorInvalidIdentifierInListAtEndOfSwitchAction = -200046

const DAQmxErrorDeviceRemoved = -200045

const DAQmxErrorRoutingPathNotAvailable = -200044

const DAQmxErrorRoutingHardwareBusy = -200043

const DAQmxErrorRequestedSignalInversionForRoutingNotPossible = -200042

const DAQmxErrorInvalidRoutingDestinationTerminalName = -200041

const DAQmxErrorInvalidRoutingSourceTerminalName = -200040

const DAQmxErrorRoutingNotSupportedForDevice = -200039

const DAQmxErrorWaitIsLastInstructionOfLoopInScript = -200038

const DAQmxErrorClearIsLastInstructionOfLoopInScript = -200037

const DAQmxErrorInvalidLoopIterationsInScript = -200036

const DAQmxErrorRepeatLoopNestingTooDeepInScript = -200035

const DAQmxErrorMarkerPositionOutsideSubsetInScript = -200034

const DAQmxErrorSubsetStartOffsetNotAlignedInScript = -200033

const DAQmxErrorInvalidSubsetLengthInScript = -200032

const DAQmxErrorMarkerPositionNotAlignedInScript = -200031

const DAQmxErrorSubsetOutsideWaveformInScript = -200030

const DAQmxErrorMarkerOutsideWaveformInScript = -200029

const DAQmxErrorWaveformInScriptNotInMem = -200028

const DAQmxErrorKeywordExpectedInScript = -200027

const DAQmxErrorBufferNameExpectedInScript = -200026

const DAQmxErrorProcedureNameExpectedInScript = -200025

const DAQmxErrorScriptHasInvalidIdentifier = -200024

const DAQmxErrorScriptHasInvalidCharacter = -200023

const DAQmxErrorResourceAlreadyReserved = -200022

const DAQmxErrorSelfTestFailed = -200020

const DAQmxErrorADCOverrun = -200019

const DAQmxErrorDACUnderflow = -200018

const DAQmxErrorInputFIFOUnderflow = -200017

const DAQmxErrorOutputFIFOUnderflow = -200016

const DAQmxErrorSCXISerialCommunication = -200015

const DAQmxErrorDigitalTerminalSpecifiedMoreThanOnce = -200014

const DAQmxErrorDigitalOutputNotSupported = -200012

const DAQmxErrorInconsistentChannelDirections = -200011

const DAQmxErrorInputFIFOOverflow = -200010

const DAQmxErrorTimeStampOverwritten = -200009

const DAQmxErrorStopTriggerHasNotOccurred = -200008

const DAQmxErrorRecordNotAvailable = -200007

const DAQmxErrorRecordOverwritten = -200006

const DAQmxErrorDataNotAvailable = -200005

const DAQmxErrorDataOverwrittenInDeviceMemory = -200004

const DAQmxErrorDuplicatedChannel = -200003

const DAQmxWarningTimestampCounterRolledOver = 200003

const DAQmxWarningInputTerminationOverloaded = 200004

const DAQmxWarningADCOverloaded = 200005

const DAQmxWarningPLLUnlocked = 200007

const DAQmxWarningCounter0DMADuringAIConflict = 200008

const DAQmxWarningCounter1DMADuringAOConflict = 200009

const DAQmxWarningStoppedBeforeDone = 200010

const DAQmxWarningRateViolatesSettlingTime = 200011

const DAQmxWarningRateViolatesMaxADCRate = 200012

const DAQmxWarningUserDefInfoStringTooLong = 200013

const DAQmxWarningTooManyInterruptsPerSecond = 200014

const DAQmxWarningPotentialGlitchDuringWrite = 200015

const DAQmxWarningDevNotSelfCalibratedWithDAQmx = 200016

const DAQmxWarningAISampRateTooLow = 200017

const DAQmxWarningAIConvRateTooLow = 200018

const DAQmxWarningReadOffsetCoercion = 200019

const DAQmxWarningPretrigCoercion = 200020

const DAQmxWarningSampValCoercedToMax = 200021

const DAQmxWarningSampValCoercedToMin = 200022

const DAQmxWarningPropertyVersionNew = 200024

const DAQmxWarningUserDefinedInfoTooLong = 200025

const DAQmxWarningCAPIStringTruncatedToFitBuffer = 200026

const DAQmxWarningSampClkRateTooLow = 200027

const DAQmxWarningPossiblyInvalidCTRSampsInFiniteDMAAcq = 200028

const DAQmxWarningRISAcqCompletedSomeBinsNotFilled = 200029

const DAQmxWarningPXIDevTempExceedsMaxOpTemp = 200030

const DAQmxWarningOutputGainTooLowForRFFreq = 200031

const DAQmxWarningOutputGainTooHighForRFFreq = 200032

const DAQmxWarningMultipleWritesBetweenSampClks = 200033

const DAQmxWarningDeviceMayShutDownDueToHighTemp = 200034

const DAQmxWarningRateViolatesMinADCRate = 200035

const DAQmxWarningSampClkRateAboveDevSpecs = 200036

const DAQmxWarningCOPrevDAQmxWriteSettingsOverwrittenForHWTimedSinglePoint = 200037

const DAQmxWarningLowpassFilterSettlingTimeExceedsUserTimeBetween2ADCConversions = 200038

const DAQmxWarningLowpassFilterSettlingTimeExceedsDriverTimeBetween2ADCConversions = 200039

const DAQmxWarningSampClkRateViolatesSettlingTimeForGen = 200040

const DAQmxWarningInvalidCalConstValueForAI = 200041

const DAQmxWarningInvalidCalConstValueForAO = 200042

const DAQmxWarningChanCalExpired = 200043

const DAQmxWarningUnrecognizedEnumValueEncounteredInStorage = 200044

const DAQmxWarningTableCRCNotCorrect = 200045

const DAQmxWarningExternalCRCNotCorrect = 200046

const DAQmxWarningSelfCalCRCNotCorrect = 200047

const DAQmxWarningDeviceSpecExceeded = 200048

const DAQmxWarningOnlyGainCalibrated = 200049

const DAQmxWarningReversePowerProtectionActivated = 200050

const DAQmxWarningOverVoltageProtectionActivated = 200051

const DAQmxWarningBufferSizeNotMultipleOfSectorSize = 200052

const DAQmxWarningSampleRateMayCauseAcqToFail = 200053

const DAQmxWarningUserAreaCRCNotCorrect = 200054

const DAQmxWarningPowerUpInfoCRCNotCorrect = 200055

const DAQmxWarningConnectionCountHasExceededRecommendedLimit = 200056

const DAQmxWarningNetworkDeviceAlreadyAdded = 200057

const DAQmxWarningAccessoryConnectionCountIsInvalid = 200058

const DAQmxWarningUnableToDisconnectPorts = 200059

const DAQmxWarningReadRepeatedData = 200060

const DAQmxWarningPXI5600_NotConfigured = 200061

const DAQmxWarningPXI5661_IncorrectlyConfigured = 200062

const DAQmxWarningPXIe5601_NotConfigured = 200063

const DAQmxWarningPXIe5663_IncorrectlyConfigured = 200064

const DAQmxWarningPXIe5663E_IncorrectlyConfigured = 200065

const DAQmxWarningPXIe5603_NotConfigured = 200066

const DAQmxWarningPXIe5665_5603_IncorrectlyConfigured = 200067

const DAQmxWarningPXIe5667_5603_IncorrectlyConfigured = 200068

const DAQmxWarningPXIe5605_NotConfigured = 200069

const DAQmxWarningPXIe5665_5605_IncorrectlyConfigured = 200070

const DAQmxWarningPXIe5667_5605_IncorrectlyConfigured = 200071

const DAQmxWarningPXIe5606_NotConfigured = 200072

const DAQmxWarningPXIe5665_5606_IncorrectlyConfigured = 200073

const DAQmxWarningPXI5610_NotConfigured = 200074

const DAQmxWarningPXI5610_IncorrectlyConfigured = 200075

const DAQmxWarningPXIe5611_NotConfigured = 200076

const DAQmxWarningPXIe5611_IncorrectlyConfigured = 200077

const DAQmxWarningUSBHotfixForDAQ = 200078

const DAQmxWarningNoChangeSupersededByIdleBehavior = 200079

const DAQmxWarningReadNotCompleteBeforeSampClk = 209800

const DAQmxWarningWriteNotCompleteBeforeSampClk = 209801

const DAQmxWarningWaitForNextSampClkDetectedMissedSampClk = 209802

const DAQmxWarningOutputDataTransferConditionNotSupported = 209803

const DAQmxWarningTimestampMayBeInvalid = 209804

const DAQmxWarningFirstSampleTimestampInaccurate = 209805

const DAQmxErrorInterfaceObsoleted_Routing = -89169

const DAQmxErrorRoCoServiceNotAvailable_Routing = -89168

const DAQmxErrorRoutingDestTermPXIDStarXNotInSystemTimingSlot_Routing = -89167

const DAQmxErrorRoutingSrcTermPXIDStarXNotInSystemTimingSlot_Routing = -89166

const DAQmxErrorRoutingSrcTermPXIDStarInNonDStarTriggerSlot_Routing = -89165

const DAQmxErrorRoutingDestTermPXIDStarInNonDStarTriggerSlot_Routing = -89164

const DAQmxErrorRoutingDestTermPXIClk10InNotInStarTriggerSlot_Routing = -89162

const DAQmxErrorRoutingDestTermPXIClk10InNotInSystemTimingSlot_Routing = -89161

const DAQmxErrorRoutingDestTermPXIStarXNotInStarTriggerSlot_Routing = -89160

const DAQmxErrorRoutingDestTermPXIStarXNotInSystemTimingSlot_Routing = -89159

const DAQmxErrorRoutingSrcTermPXIStarXNotInStarTriggerSlot_Routing = -89158

const DAQmxErrorRoutingSrcTermPXIStarXNotInSystemTimingSlot_Routing = -89157

const DAQmxErrorRoutingSrcTermPXIStarInNonStarTriggerSlot_Routing = -89156

const DAQmxErrorRoutingDestTermPXIStarInNonStarTriggerSlot_Routing = -89155

const DAQmxErrorRoutingDestTermPXIStarInStarTriggerSlot_Routing = -89154

const DAQmxErrorRoutingDestTermPXIStarInSystemTimingSlot_Routing = -89153

const DAQmxErrorRoutingSrcTermPXIStarInStarTriggerSlot_Routing = -89152

const DAQmxErrorRoutingSrcTermPXIStarInSystemTimingSlot_Routing = -89151

const DAQmxErrorInvalidSignalModifier_Routing = -89150

const DAQmxErrorRoutingDestTermPXIClk10InNotInSlot2_Routing = -89149

const DAQmxErrorRoutingDestTermPXIStarXNotInSlot2_Routing = -89148

const DAQmxErrorRoutingSrcTermPXIStarXNotInSlot2_Routing = -89147

const DAQmxErrorRoutingSrcTermPXIStarInSlot16AndAbove_Routing = -89146

const DAQmxErrorRoutingDestTermPXIStarInSlot16AndAbove_Routing = -89145

const DAQmxErrorRoutingDestTermPXIStarInSlot2_Routing = -89144

const DAQmxErrorRoutingSrcTermPXIStarInSlot2_Routing = -89143

const DAQmxErrorRoutingDestTermPXIChassisNotIdentified_Routing = -89142

const DAQmxErrorRoutingSrcTermPXIChassisNotIdentified_Routing = -89141

const DAQmxErrorTrigLineNotFoundSingleDevRoute_Routing = -89140

const DAQmxErrorNoCommonTrigLineForRoute_Routing = -89139

const DAQmxErrorResourcesInUseForRouteInTask_Routing = -89138

const DAQmxErrorResourcesInUseForRoute_Routing = -89137

const DAQmxErrorRouteNotSupportedByHW_Routing = -89136

const DAQmxErrorResourcesInUseForInversionInTask_Routing = -89135

const DAQmxErrorResourcesInUseForInversion_Routing = -89134

const DAQmxErrorInversionNotSupportedByHW_Routing = -89133

const DAQmxErrorResourcesInUseForProperty_Routing = -89132

const DAQmxErrorRouteSrcAndDestSame_Routing = -89131

const DAQmxErrorDevAbsentOrUnavailable_Routing = -89130

const DAQmxErrorInvalidTerm_Routing = -89129

const DAQmxErrorCannotTristateTerm_Routing = -89128

const DAQmxErrorCannotTristateBusyTerm_Routing = -89127

const DAQmxErrorCouldNotReserveRequestedTrigLine_Routing = -89126

const DAQmxErrorTrigLineNotFound_Routing = -89125

const DAQmxErrorRoutingPathNotAvailable_Routing = -89124

const DAQmxErrorRoutingHardwareBusy_Routing = -89123

const DAQmxErrorRequestedSignalInversionForRoutingNotPossible_Routing = -89122

const DAQmxErrorInvalidRoutingDestinationTerminalName_Routing = -89121

const DAQmxErrorInvalidRoutingSourceTerminalName_Routing = -89120

const DAQmxErrorServiceLocatorNotAvailable_Routing = -88907

const DAQmxErrorCouldNotConnectToServer_Routing = -88900

const DAQmxErrorDeviceNameContainsSpacesOrPunctuation_Routing = -88720

const DAQmxErrorDeviceNameContainsNonprintableCharacters_Routing = -88719

const DAQmxErrorDeviceNameIsEmpty_Routing = -88718

const DAQmxErrorDeviceNameNotFound_Routing = -88717

const DAQmxErrorLocalRemoteDriverVersionMismatch_Routing = -88716

const DAQmxErrorDuplicateDeviceName_Routing = -88715

const DAQmxErrorRuntimeAborting_Routing = -88710

const DAQmxErrorRuntimeAborted_Routing = -88709

const DAQmxErrorResourceNotInPool_Routing = -88708

const DAQmxErrorDriverDeviceGUIDNotFound_Routing = -88705

const DAQmxErrorPALUSBTransactionError = -50808

const DAQmxErrorPALIsocStreamBufferError = -50807

const DAQmxErrorPALInvalidAddressComponent = -50806

const DAQmxErrorPALSharingViolation = -50805

const DAQmxErrorPALInvalidDeviceState = -50804

const DAQmxErrorPALConnectionReset = -50803

const DAQmxErrorPALConnectionAborted = -50802

const DAQmxErrorPALConnectionRefused = -50801

const DAQmxErrorPALBusResetOccurred = -50800

const DAQmxErrorPALWaitInterrupted = -50700

const DAQmxErrorPALMessageUnderflow = -50651

const DAQmxErrorPALMessageOverflow = -50650

const DAQmxErrorPALThreadAlreadyDead = -50604

const DAQmxErrorPALThreadStackSizeNotSupported = -50603

const DAQmxErrorPALThreadControllerIsNotThreadCreator = -50602

const DAQmxErrorPALThreadHasNoThreadObject = -50601

const DAQmxErrorPALThreadCouldNotRun = -50600

const DAQmxErrorPALSyncAbandoned = -50551

const DAQmxErrorPALSyncTimedOut = -50550

const DAQmxErrorPALReceiverSocketInvalid = -50503

const DAQmxErrorPALSocketListenerInvalid = -50502

const DAQmxErrorPALSocketListenerAlreadyRegistered = -50501

const DAQmxErrorPALDispatcherAlreadyExported = -50500

const DAQmxErrorPALDMALinkEventMissed = -50450

const DAQmxErrorPALBusError = -50413

const DAQmxErrorPALRetryLimitExceeded = -50412

const DAQmxErrorPALTransferOverread = -50411

const DAQmxErrorPALTransferOverwritten = -50410

const DAQmxErrorPALPhysicalBufferFull = -50409

const DAQmxErrorPALPhysicalBufferEmpty = -50408

const DAQmxErrorPALLogicalBufferFull = -50407

const DAQmxErrorPALLogicalBufferEmpty = -50406

const DAQmxErrorPALTransferAborted = -50405

const DAQmxErrorPALTransferStopped = -50404

const DAQmxErrorPALTransferInProgress = -50403

const DAQmxErrorPALTransferNotInProgress = -50402

const DAQmxErrorPALCommunicationsFault = -50401

const DAQmxErrorPALTransferTimedOut = -50400

const DAQmxErrorPALMemoryHeapNotEmpty = -50355

const DAQmxErrorPALMemoryBlockCheckFailed = -50354

const DAQmxErrorPALMemoryPageLockFailed = -50353

const DAQmxErrorPALMemoryFull = -50352

const DAQmxErrorPALMemoryAlignmentFault = -50351

const DAQmxErrorPALMemoryConfigurationFault = -50350

const DAQmxErrorPALDeviceInitializationFault = -50303

const DAQmxErrorPALDeviceNotSupported = -50302

const DAQmxErrorPALDeviceUnknown = -50301

const DAQmxErrorPALDeviceNotFound = -50300

const DAQmxErrorPALFeatureDisabled = -50265

const DAQmxErrorPALComponentBusy = -50264

const DAQmxErrorPALComponentAlreadyInstalled = -50263

const DAQmxErrorPALComponentNotUnloadable = -50262

const DAQmxErrorPALComponentNeverLoaded = -50261

const DAQmxErrorPALComponentAlreadyLoaded = -50260

const DAQmxErrorPALComponentCircularDependency = -50259

const DAQmxErrorPALComponentInitializationFault = -50258

const DAQmxErrorPALComponentImageCorrupt = -50257

const DAQmxErrorPALFeatureNotSupported = -50256

const DAQmxErrorPALFunctionNotFound = -50255

const DAQmxErrorPALFunctionObsolete = -50254

const DAQmxErrorPALComponentTooNew = -50253

const DAQmxErrorPALComponentTooOld = -50252

const DAQmxErrorPALComponentNotFound = -50251

const DAQmxErrorPALVersionMismatch = -50250

const DAQmxErrorPALFileFault = -50209

const DAQmxErrorPALFileWriteFault = -50208

const DAQmxErrorPALFileReadFault = -50207

const DAQmxErrorPALFileSeekFault = -50206

const DAQmxErrorPALFileCloseFault = -50205

const DAQmxErrorPALFileOpenFault = -50204

const DAQmxErrorPALDiskFull = -50203

const DAQmxErrorPALOSFault = -50202

const DAQmxErrorPALOSInitializationFault = -50201

const DAQmxErrorPALOSUnsupported = -50200

const DAQmxErrorPALCalculationOverflow = -50175

const DAQmxErrorPALHardwareFault = -50152

const DAQmxErrorPALFirmwareFault = -50151

const DAQmxErrorPALSoftwareFault = -50150

const DAQmxErrorPALMessageQueueFull = -50108

const DAQmxErrorPALResourceAmbiguous = -50107

const DAQmxErrorPALResourceBusy = -50106

const DAQmxErrorPALResourceInitialized = -50105

const DAQmxErrorPALResourceNotInitialized = -50104

const DAQmxErrorPALResourceReserved = -50103

const DAQmxErrorPALResourceNotReserved = -50102

const DAQmxErrorPALResourceNotAvailable = -50101

const DAQmxErrorPALResourceOwnedBySystem = -50100

const DAQmxErrorPALBadToken = -50020

const DAQmxErrorPALBadThreadMultitask = -50019

const DAQmxErrorPALBadLibrarySpecifier = -50018

const DAQmxErrorPALBadAddressSpace = -50017

const DAQmxErrorPALBadWindowType = -50016

const DAQmxErrorPALBadAddressClass = -50015

const DAQmxErrorPALBadWriteCount = -50014

const DAQmxErrorPALBadWriteOffset = -50013

const DAQmxErrorPALBadWriteMode = -50012

const DAQmxErrorPALBadReadCount = -50011

const DAQmxErrorPALBadReadOffset = -50010

const DAQmxErrorPALBadReadMode = -50009

const DAQmxErrorPALBadCount = -50008

const DAQmxErrorPALBadOffset = -50007

const DAQmxErrorPALBadMode = -50006

const DAQmxErrorPALBadDataSize = -50005

const DAQmxErrorPALBadPointer = -50004

const DAQmxErrorPALBadSelector = -50003

const DAQmxErrorPALBadDevice = -50002

const DAQmxErrorPALIrrelevantAttribute = -50001

const DAQmxErrorPALValueConflict = -50000

const DAQmxWarningPALValueConflict = 50000

const DAQmxWarningPALIrrelevantAttribute = 50001

const DAQmxWarningPALBadDevice = 50002

const DAQmxWarningPALBadSelector = 50003

const DAQmxWarningPALBadPointer = 50004

const DAQmxWarningPALBadDataSize = 50005

const DAQmxWarningPALBadMode = 50006

const DAQmxWarningPALBadOffset = 50007

const DAQmxWarningPALBadCount = 50008

const DAQmxWarningPALBadReadMode = 50009

const DAQmxWarningPALBadReadOffset = 50010

const DAQmxWarningPALBadReadCount = 50011

const DAQmxWarningPALBadWriteMode = 50012

const DAQmxWarningPALBadWriteOffset = 50013

const DAQmxWarningPALBadWriteCount = 50014

const DAQmxWarningPALBadAddressClass = 50015

const DAQmxWarningPALBadWindowType = 50016

const DAQmxWarningPALBadThreadMultitask = 50019

const DAQmxWarningPALResourceOwnedBySystem = 50100

const DAQmxWarningPALResourceNotAvailable = 50101

const DAQmxWarningPALResourceNotReserved = 50102

const DAQmxWarningPALResourceReserved = 50103

const DAQmxWarningPALResourceNotInitialized = 50104

const DAQmxWarningPALResourceInitialized = 50105

const DAQmxWarningPALResourceBusy = 50106

const DAQmxWarningPALResourceAmbiguous = 50107

const DAQmxWarningPALFirmwareFault = 50151

const DAQmxWarningPALHardwareFault = 50152

const DAQmxWarningPALOSUnsupported = 50200

const DAQmxWarningPALOSFault = 50202

const DAQmxWarningPALFunctionObsolete = 50254

const DAQmxWarningPALFunctionNotFound = 50255

const DAQmxWarningPALFeatureNotSupported = 50256

const DAQmxWarningPALComponentInitializationFault = 50258

const DAQmxWarningPALComponentAlreadyLoaded = 50260

const DAQmxWarningPALComponentNotUnloadable = 50262

const DAQmxWarningPALMemoryAlignmentFault = 50351

const DAQmxWarningPALMemoryHeapNotEmpty = 50355

const DAQmxWarningPALTransferNotInProgress = 50402

const DAQmxWarningPALTransferInProgress = 50403

const DAQmxWarningPALTransferStopped = 50404

const DAQmxWarningPALTransferAborted = 50405

const DAQmxWarningPALLogicalBufferEmpty = 50406

const DAQmxWarningPALLogicalBufferFull = 50407

const DAQmxWarningPALPhysicalBufferEmpty = 50408

const DAQmxWarningPALPhysicalBufferFull = 50409

const DAQmxWarningPALTransferOverwritten = 50410

const DAQmxWarningPALTransferOverread = 50411

const DAQmxWarningPALDispatcherAlreadyExported = 50500

const DAQmxWarningPALSyncAbandoned = 50551

# exports
# const PREFIXES = ["CX", "DAQmx"]
# for name in names(@__MODULE__; all=true), prefix in PREFIXES
#     if startswith(string(name), prefix)
#         @eval export $name
#     end
# end

# end # module

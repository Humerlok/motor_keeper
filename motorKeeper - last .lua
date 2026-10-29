--Code by Renan Mandelo Oliveira (https://github.com/Humerlok)

--variáveis
local LOOP_TIME = 100 -- tempo do loop principal em milissegundos
local ANGLE_FAILURE_INTERVAL = 500 -- tempo continuo (ms) acima do limite para confirmar falha de angulo
local ANGLE_THRESHOLD = param:get("SCR_USER2") -- ângulo de inclinação (roll/pitch) que indica perda de controle
local HEADROOM_CUSTOM = param:get("SCR_USER3") -- valor personalizado para o headroom do motor

local roll = 0
local pitch = 0
local motorDown = false
local failureMotor = -1
local thrustBurst = false
local wasParamChanged = false
local isDefaultParameters = true
local isFlying = false
local angle_fail = false

--variáveis de tempo
local now = 0
local last_msg_time = 0
local MSG_INTERVAL = 3000 --intervalo entre as menssagens
local angle_exceeded_since = nil -- instante em que o angulo passou do limite (nil = dentro do limite)


-- parametros que serão alterados em caso de falha de motor
local PARAMETERS_NAMES = {
    "ANGLE_MAX",
    "MOT_YAW_HEADROOM",
    "PLND_ENABLED",
    "WPNAV_SPEED",
    "WPNAV_RADIUS",
    "WPNAV_ACCEL",
    "WP_YAW_BEHAVIOR",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_IMAX",
    "ATC_RAT_YAW_D",
    "ATC_ANG_YAW_P",
    "ATC_ACCEL_Y_MAX",
    "ATC_RATE_Y_MAX"
}

-- armazenará os valores iniciais dos parâmetros. Será preenchida pela função getDefaultParams()
local PARAMETERS_DEFAULT = {}

-- novos valores dos parâmetros, caso a aeronave perca um motor. Será preenchida pela função calculateParamsNewValue()
local PARAMETERS_NEW_VALUE = {}

--níveis de mensagem para o GCS
local MSG_LVL = {
    EMERGENCY   = 0,
    ALERT       = 1,
    CRITICAL    = 2,
    ERROR       = 3,
    WARNING     = 4,
    NOTICE      = 5,
    INFO        = 6,
    DEBUG       = 7
}

----funções para escrita e leitura de parâmetros-----------------------------------------------------------------------------------------------------------------------------------------------------

-- função para verificar se uma string é vazia ou nula
local function isEmpty(s)
    return s == nil or s == ''
end

-- retorna fator * valor padrão, ou nil se o valor padrão não existir (parâmetro não encontrado)
local function scaledDefault(paramName, factor)
    local default = PARAMETERS_DEFAULT[paramName]
    if isEmpty(default) then
        return nil
    end
    return factor * default
end

-- define os novos valores de parâmetros a serem aplicados caso a aeronave perca um motor
local function calculateParamsNewValue()
    PARAMETERS_NEW_VALUE["ANGLE_MAX"]           = 2000  -- "ANGLE_MAX" - centigraus (2000 = 20 graus)
    PARAMETERS_NEW_VALUE["MOT_YAW_HEADROOM"]    = HEADROOM_CUSTOM     -- "MOT_YAW_HEADROOM"
    PARAMETERS_NEW_VALUE["WPNAV_SPEED"]         = 1100  -- "WPNAV_SPEED"
    PARAMETERS_NEW_VALUE["WPNAV_RADIUS"]        = 750   -- "WPNAV_RADIUS"
    PARAMETERS_NEW_VALUE["WPNAV_ACCEL"]         = 50    -- "WPNAV_ACCEL"
    PARAMETERS_NEW_VALUE["PLND_ENABLED"]        = 0     -- "PLND_ENABLED"
    PARAMETERS_NEW_VALUE["WP_YAW_BEHAVIOR"]     = 0     -- "WP_YAW_BEHAVIOR"
    PARAMETERS_NEW_VALUE["ATC_RAT_YAW_P"]       = 0.15   -- "ATC_RAT_YAW_P"
    PARAMETERS_NEW_VALUE["ATC_RAT_YAW_I"]       = 0  -- "ATC_RAT_YAW_I"
    PARAMETERS_NEW_VALUE["ATC_RAT_YAW_IMAX"]    = 0     -- "ATC_RAT_YAW_IMAX"
    PARAMETERS_NEW_VALUE["ATC_RAT_YAW_D"]       = 0     -- "ATC_RAT_YAW_D"
    PARAMETERS_NEW_VALUE["ATC_ANG_YAW_P"]       = 0.5   -- "ATC_ANG_YAW_P"
    PARAMETERS_NEW_VALUE["ATC_ACCEL_Y_MAX"] = scaledDefault("ATC_ACCEL_Y_MAX", 0.5) -- "ATC_ACCEL_Y_MAX"
    PARAMETERS_NEW_VALUE["ATC_RATE_Y_MAX"]  = scaledDefault("ATC_RATE_Y_MAX", 0.7)  -- "ATC_RATE_Y_MAX"
end

-- função que pega os valores iniciais dos parâmetros e coloca na tabela PARAMETERS_DEFAULT
local function getDefaultParams()
    -- preenche a tabela PARAMETERS_DEFAULT com os parâmetros iniciais
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        PARAMETERS_DEFAULT[paramName] = param:get(paramName)
        if (isEmpty(PARAMETERS_DEFAULT[paramName]))then
            gcs:send_text(MSG_LVL.ERROR, string.format("[motorKeeper] %s NOT FOUND", paramName))
        else
            gcs:send_text(MSG_LVL.INFO, string.format("[motorKeeper] %s default value is %f", paramName, PARAMETERS_DEFAULT[paramName]))
        end
    end
end

-- função que retorna para os valores iniciais (não usada no momento)
local function setDefaultParams()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        local defaultValue = PARAMETERS_DEFAULT[paramName]
        if (not isEmpty(defaultValue)) then
            param:set(paramName, defaultValue)
            gcs:send_text(MSG_LVL.INFO, string.format("[motorKeeper] Changing %s to DEFAULT value: %f", paramName, defaultValue))
        end
    end
    isDefaultParameters = true
end

-- função que altera os parâmetros para os valores novos
local function setParamsNewValue()
    for index, paramName in ipairs(PARAMETERS_NAMES) do
        local newValue = PARAMETERS_NEW_VALUE[paramName]
        if (not isEmpty(newValue)) then
            param:set(paramName, newValue)
        end
    end
    isDefaultParameters = false
end

----funções de detecção de falha------------------------------------------------------------------------------------------------------------------------------------------------------

-- detecção por perda de motor (thrust burst)
local function thrust_failure()

    thrustBurst = MotorsMatrix:get_thrust_boost()

    if thrustBurst then
        failureMotor = MotorsMatrix:get_lost_motor()
    end

    if failureMotor ~= -1 then
        return true
    else
        return false
    end
end

-- detecção pór angulo de inclinação (roll/pitch) maior que xx graus, o que indica perda de controle
local function angle_failure()

    roll = math.deg(ahrs:get_roll())
    pitch = math.deg(ahrs:get_pitch())

    if math.abs(roll) > ANGLE_THRESHOLD or math.abs(pitch) > ANGLE_THRESHOLD then
        return true
    else
        return false
    end

end

-- função principal de detecção de falha
local function failureDetector()

    if angle_failure() then
        now = millis()
        if angle_exceeded_since == nil then
            angle_exceeded_since = now
        elseif now - angle_exceeded_since >= ANGLE_FAILURE_INTERVAL then
            angle_fail = true -- latch: nunca volta a false (intencional)
        end
    else
        angle_exceeded_since = nil -- angulo voltou ao normal, descarta o pico
    end

    if thrust_failure() or angle_fail then
        return true
    else
        return false
    end

end
----Modo debug--------------------------------------------------------------------------------------------------------------------------------------------------
local function debug()
    now = millis()
    if now - last_msg_time >= MSG_INTERVAL then
        gcs:send_text(MSG_LVL.DEBUG, string.format("[motorKeeper] thrustBurst: %s", tostring(thrustBurst)))
        gcs:send_text(MSG_LVL.DEBUG, string.format("[motorKeeper] failureMotor: %d", failureMotor))
        gcs:send_text(MSG_LVL.DEBUG, string.format("[motorKeeper] isDefaultParameters: %s", tostring(isDefaultParameters)))
        gcs:send_text(MSG_LVL.DEBUG, string.format("[motorKeeper] ROLL: %f", tostring(roll)))
        gcs:send_text(MSG_LVL.DEBUG, string.format("[motorKeeper] PITCH: %f", tostring(pitch)))
        last_msg_time = now
    end
end

----loop principal------------------------------------------------------------------------------------------------------------------------------------------------------
function motorKeeper()

    local DEBUG_MODE = param:get("SCR_USER1") -- modo debug

    if DEBUG_MODE == 1 then
        debug()
    end

    local RETURN_PARAMS = param:get("SCR_USER4") -- retorna os parâmetros para os valores iniciais

    if RETURN_PARAMS  == 1 and not isDefaultParameters then
        setDefaultParams()

        motorDown = false
        failureMotor = -1
        thrustBurst = false
        wasParamChanged = false
        isDefaultParameters = true
        isFlying = false
        angle_fail = false

        gcs:send_text(MSG_LVL.INFO, "[motorKeeper] Parameters returned to default values")

        return motorKeeper, LOOP_TIME
    end

    isFlying = vehicle:get_likely_flying()

    --se tiver voando ele roda tudo
    if isFlying then

        motorDown = failureDetector()

        if motorDown and isDefaultParameters then

            if failureMotor ~= -1 then
                gcs:send_text(MSG_LVL.EMERGENCY, "[motorKeeper] Falha no motor:" .. tostring(failureMotor + 1))
            else
                gcs:send_text(MSG_LVL.EMERGENCY, "[motorKeeper] Falha detectada por angulo de inclinacao")
            end
            
            setParamsNewValue()
        end

    end

    return motorKeeper, LOOP_TIME
end
------------------------------------------------------------------------------------------------------------------------------------------------------
--inicio do script
gcs:send_text(MSG_LVL.INFO, "motorKeeper v1.2.2 loaded")
getDefaultParams()
calculateParamsNewValue()
return motorKeeper(), 1000
------------------------------------------------------------------------------------------------------------------------------------------------------
--Nossa Senhora das 6 hélices, protetora dos drones,
--que atravessam a cidade as 6 horas da tarde,
--Fazei com que eu chegue do outro lado,
--sem perda de motor.
--Amém.

--code by Renan Mandelo Oliveira
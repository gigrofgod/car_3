using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Weight distribution")]
    [SerializeField, Range(0, 1)] private float _frontAxisShare = 0.5f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;


    [Header("Handbrake")]
    [SerializeField] private KeyCode handbrakeKey = KeyCode.Space;
    [SerializeField] private float handbrakeBrakeForce = 6000f;

    private InputAction _moveAction;
    private float _throttleInput;
    private float _steepInput;
    private bool _handbrakePressed;
    private float _frontLeftNormalForce, _frontRightNormalForce, _rearLeftNormalForce, _rearRightNormalForce;
    private Rigidbody _rigidbody;
    private Vector3 g = Physics.gravity;

    [SerializeField] private float engineTorque = 400f;
    [SerializeField] private float wheelRadius = 0.3f;
    [SerializeField] private float maxSpeed = 20;

    [Header("Steering")]
    [SerializeField] private float maxSteeringAngle;
    private Quaternion frontLeftInitialRot;
    private Quaternion frontRightInitialRot;

    [Header("Tyre friction")]
    [SerializeField] private float frictionCoefficient = 1f;
    [SerializeField] private float lateralStiffnes = 80f;
    [SerializeField] private float rollingResistance;

    
    private float _speedAlongForward = 0f;
    private float _rearFxTotal = 0f; 
    private float _frontFyTotal = 0f;
    private float _fl_vLat = 0f;
    private float _fr_vLat = 0f;
    private float _rl_vLat = 0f;
    private float _rr_vLat = 0f;
  
    private float Fx = 0f;
    private float Fy = 0f;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();
        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_import) Initialize();

        frontLeftInitialRot = _frontLeftWheel.localRotation;
        frontRightInitialRot = _frontRightWheel.localRotation;
        ComputeStaticWheelLoad();
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
            frictionCoefficient = _kartConfig.frictionCoefficient;
            rollingResistance = _kartConfig.rollingResistance;
            maxSteeringAngle = _kartConfig.maxSteerAngle;
            _gearRatio = _kartConfig.gearRatio;
            wheelRadius = _kartConfig.wheelRadius;
            lateralStiffnes = _kartConfig.lateralStiffness;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steepInput = Mathf.Clamp(move.x, -1, 1);
        _throttleInput = Mathf.Clamp(move.y, -1, 1);

        _handbrakePressed = Input.GetKey(handbrakeKey);
    }

    void RotateFrontWheels()
    {
        float steerAngle = maxSteeringAngle * _steepInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = frontLeftInitialRot * steerRot;
        _frontRightWheel.localRotation = frontRightInitialRot * steerRot;
    }

    void ComputeStaticWheelLoad()
    {
        float mass = _rigidbody.mass;
        float totalWeight = mass * Mathf.Abs(g.y);
        float frontWeight = totalWeight * _frontAxisShare;
        float rearWeight = totalWeight - frontWeight;
        _frontRightNormalForce = frontWeight * 0.5f;
        _frontLeftNormalForce = _frontRightNormalForce;
        _rearRightNormalForce = rearWeight * 0.5f;
        _rearLeftNormalForce = _rearRightNormalForce;
    }

    private void ApplyEngineForces()
    {
        
        Vector3 forward = transform.forward;
        float speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, forward);
        if (_throttleInput > 0 && speedAlongForward > maxSpeed) return;

        float driveTorque = engineTorque * _throttleInput;
        float driveForcePerWheel = driveTorque / wheelRadius / 2;
        Vector3 forceRear = forward * driveForcePerWheel;

        _rigidbody.AddForceAtPosition(forceRear, _rearLeftWheel.position, ForceMode.Force);
        _rigidbody.AddForceAtPosition(forceRear, _rearRightWheel.position, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        ApplyEngineForces();

        
        _rearFxTotal = 0f;
        _frontFyTotal = 0f;

        
        ApplyWheelForce(_frontLeftWheel, _frontLeftNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_frontRightWheel, _frontRightNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_rearLeftWheel, _rearLeftNormalForce, isSteer: false, isDrive: true);
        ApplyWheelForce(_rearRightWheel, _rearRightNormalForce, isSteer: false, isDrive: true);
    }

    void ApplyWheelForce(Transform wheel, float normalForce, bool isSteer, bool isDrive)
    {
        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 velocity = _rigidbody.GetPointVelocity(wheelPos);

        float vlong = Vector3.Dot(velocity, wheelForward);
        float vlat = Vector3.Dot(velocity, wheelRight); 

        
        if (wheel == _frontLeftWheel) _fl_vLat = vlat;
        else if (wheel == _frontRightWheel) _fr_vLat = vlat;
        else if (wheel == _rearLeftWheel) _rl_vLat = vlat;
        else if (wheel == _rearRightWheel) _rr_vLat = vlat;

        Fx = 0f;
        Fy = 0f;

        if (isDrive)
        {
            _speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
            float engineTorqueOut = _engine.Simulate(_throttleInput, _speedAlongForward, Time.fixedDeltaTime);
            float totalWheelTorque = engineTorqueOut * _gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f;
            Fx += wheelTorque / wheelRadius;

            if (_handbrakePressed)
            {
                float brakeDir = vlong > 0 ? -1f : (vlong < 0 ? 1f : -1f);
                Fx += brakeDir * handbrakeBrakeForce;
            }

          
            _rearFxTotal += Fx;
        }
        else if (isSteer)
        {
            float rolling = -rollingResistance * vlong;
            Fx += rolling;

           
            _frontFyTotal += Fy;
        }

        float fyRaw = -lateralStiffnes * vlat;
        Fy += fyRaw;

       
        if (isSteer)
        {
           
        }

        float frictionlimit = frictionCoefficient * normalForce;
        float forceLenght = Mathf.Sqrt(Fx * Fx + Fy * Fy);

       
        if (forceLenght > frictionlimit)
        {
            float scale = frictionlimit / forceLenght;
            Fy += scale; 
            Fx += scale; 
        }

       
        if (isSteer)
        {
            _frontFyTotal += Fy;
        }

      

        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rigidbody.AddForceAtPosition(force, wheel.position, ForceMode.Force);
    }

    void OnGUI()
    {
       
        GUIStyle backgroundStyle = new GUIStyle();
        backgroundStyle.normal.background = MakeTex(2, 2, new Color(0.1f, 0.1f, 0.2f, 0.85f));
        backgroundStyle.padding = new RectOffset(15, 15, 15, 15);

        GUIStyle headerStyle = new GUIStyle();
        headerStyle.fontSize = 18;
        headerStyle.fontStyle = FontStyle.Bold;
        headerStyle.normal.textColor = new Color(1f, 0.8f, 0.2f);
        headerStyle.alignment = TextAnchor.MiddleCenter;

        GUIStyle textStyle = new GUIStyle();
        textStyle.fontSize = 16;
        textStyle.fontStyle = FontStyle.Bold;
        textStyle.normal.textColor = Color.white;

        GUIStyle speedStyle = new GUIStyle();
        speedStyle.fontSize = 50;
        speedStyle.fontStyle = FontStyle.Bold;
        speedStyle.normal.textColor = new Color(0.2f, 1f, 0.4f);
        speedStyle.alignment = TextAnchor.MiddleCenter;

       
        GUILayout.BeginArea(new Rect(30, 30, 450, 450), backgroundStyle);

        GUILayout.Label("KART DASHBOARD", headerStyle);
        GUILayout.Space(10);

       
        float speedKmh = _speedAlongForward * 3.6f;
        GUILayout.Label($"{(speedKmh):0}", speedStyle);
        GUILayout.Label($"m/s: {_speedAlongForward:0.1} | km/h", headerStyle);
        GUILayout.Space(20);

        
        GUILayout.Label("ENGINE", headerStyle);
        GUILayout.BeginVertical("box");
        GUILayout.Label($"RPM:     {_engine.CurrentRpm:0}", textStyle);
        if (_engine.CurrentTorque > 350) GUI.color = new Color(1f, 0.4f, 0.4f);
      
        GUILayout.Label($"Torque:  {_engine.CurrentTorque:0} N·m", textStyle);
        GUI.color = Color.white;
        GUILayout.EndVertical();

        GUILayout.Space(15);

       
        GUILayout.Label("AXLE FORCES", headerStyle);
        GUILayout.BeginVertical("box");
     
        GUILayout.Label($"Rear Fx Total: {_rearFxTotal:0.0} N", textStyle);
     
        GUILayout.Label($"Front Fy Total: {_frontFyTotal:0.0} N", textStyle);
        GUILayout.EndVertical();

        GUILayout.Space(15);

       
        GUILayout.Label("WHEEL SLIP/SLIDE (v_lat sign)", headerStyle);
        GUILayout.BeginVertical("box");

     
        GUILayout.Label($"FL v_lat: {_fl_vLat:0.2} ({(_fl_vLat > 0 ? "Right" : (_fl_vLat < 0 ? "Left" : "—"))})", textStyle);
        GUILayout.Label($"FR v_lat: {_fr_vLat:0.2} ({(_fr_vLat > 0 ? "Right" : (_fr_vLat < 0 ? "Left" : "—"))})", textStyle);
        GUILayout.Label($"RL v_lat: {_rl_vLat:0.2} ({(_rl_vLat > 0 ? "Right" : (_rl_vLat < 0 ? "Left" : "—"))})", textStyle);
        GUILayout.Label($"RR v_lat: {_rr_vLat:0.2} ({(_rr_vLat > 0 ? "Right" : (_rr_vLat < 0 ? "Left" : "—"))})", textStyle);

        GUILayout.EndVertical();

        GUILayout.EndArea();
    }

    
    private Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; ++i) pix[i] = col;
        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        return result;
    }
}

namespace RobotRaconteur.Companion.Robot


    public struct JointTrajectoryLimits
    
        public double[] j_max  get; set; 
        public double[] a_max  get; set; 
        public double[] v_max  get; set; 
        public double[] x_min  get; set; 
        public double[] x_max  get; set; 
    

    public struct JointTrajectoryPositionRequest
    
        public double[] current_position;
        public double[] current_velocity;
        public double[] desired_position;
        public double[] desired_velocity;

        public double[] max_velocity;

        public double? desired_time;
        public double speed_ratio;
        public double? splice_time;
    ;

    public struct JointTrajectoryVelocityRequest
    
        public double[] current_position;
        public double[] current_velocity;
        public double[] desired_velocity;

        public double timeout;
        public double speed_ratio;        
    ;

    public struct JointTrajectoryPositionCommand
    
        public double[] command_position;
        public double[] command_velocity;
    

    public abstract class JointTrajectoryGenerator
    
        protected uint _joint_count;

        protected JointTrajectoryLimits _limits;
       
        protected double _speed_ratio = 1.0;
        protected double? _t_des;

        virtual public double? T_des
        
            get => _t_des;
        

        virtual protected double SpeedRatio
        
            get => _speed_ratio;
        

        abstract protected double[] TargetPosition  get; 

        abstract protected double[] TargetVelocity  get; 
        
        protected JointTrajectoryGenerator(uint joint_count, ref JointTrajectoryLimits limits)
        
            _joint_count = joint_count;
            _limits = limits;
        

        public abstract bool UpdateDesiredPosition(ref JointTrajectoryPositionRequest request );

        public abstract bool UpdateDesiredVelocity(ref JointTrajectoryVelocityRequest request);

        public abstract bool GetCommand(double t, out JointTrajectoryPositionCommand command);
        public abstract double T_Final  get; 

        public abstract bool IsValid  get; 

    


    public class TrapezoidalJointTrajectoryGenerator : JointTrajectoryGenerator
    

        internal TrapezoidalJointTrajectoryGeneraterExec exec;

        public TrapezoidalJointTrajectoryGenerator(uint joint_count, ref JointTrajectoryLimits limits)
            : base(joint_count, ref limits)
        

        

        public override double T_Final => exec?.t_final ?? 0.0;

        public override bool IsValid => exec != null;

        protected override double[] TargetPosition => exec?.xf;

        protected override double[] TargetVelocity => exec?.v3;

        public override bool GetCommand(double t, out JointTrajectoryPositionCommand command)
        
            command = new JointTrajectoryPositionCommand();
            if (exec == null)
            
                
                return false;
            

            command.command_position = new double[_joint_count];
            command.command_velocity = new double[_joint_count];
            return exec.CalcAtTime(t, ref command.command_position, ref command.command_velocity);
        

        public override bool UpdateDesiredPosition(ref JointTrajectoryPositionRequest request)
        
            if (request.desired_time != null)
            
                throw new InvalidOperationException("desired_time not supported");
            

            exec = TrapezoidalJointTrajectoryGeneraterCalc.InitializePosExec(_joint_count, _limits, request);
            return true;
        

        public override bool UpdateDesiredVelocity(ref JointTrajectoryVelocityRequest request)
        
            exec = TrapezoidalJointTrajectoryGeneraterCalc.InitializeVelExec(_joint_count, _limits, request);
            return true;
        
    


    static class TrapezoidalJointTrajectoryGeneraterCalc
    
        public static TrapezoidalJointTrajectoryGeneraterExec InitializePosExec(uint joint_count, JointTrajectoryLimits limits, JointTrajectoryPositionRequest request)
        
            Debug.Assert(request.desired_time == null);

            double[] a_max = (double[])limits.a_max.Clone();
            double[] v_max = (double[])limits.v_max.Clone();
            if (request.speed_ratio != 0.0)
            
                for (int i = 0; i < joint_count; i++)
                
                    a_max[i] = a_max[i] * request.speed_ratio;
                    v_max[i] = v_max[i] * request.speed_ratio;

                    if (request.max_velocity != null)
                    
                        double req_vel = request.max_velocity[i] * request.speed_ratio;
                        if (req_vel < v_max[i])
                        
                            v_max[i] = req_vel;
                        
                        if (req_vel > v_max[i])
                        
                            throw new ArgumentException("req_vel must be less than or equal to v_max ");
                        

                    
                
            
            else
            
                for (int i = 0; i < joint_count; i++)
                
                    if (request.max_velocity[i] > v_max[i])
                    
                        throw new ArgumentException("req_vel must be less than or equal to v_max ");
                    

                    if (request.max_velocity[i] < v_max[i])
                    
                        v_max[i] = request.max_velocity[i];
                    
                
            

            double[] dx = new double[joint_count];
            for (int i = 0; i < joint_count; i++)
            
                dx[i] = request.desired_position[i] - request.current_position[i];
            

            double[] t1 = new double[joint_count];
            double[] t2 = new double[joint_count];
            double[] t3 = new double[joint_count];

            double[] v1 = new double[joint_count];
            double[] a1 = new double[joint_count];
            double[] a3 = new double[joint_count];

            for (int i = 0; i < joint_count; i++)
            
                if (dx[i] == 0 && request.desired_velocity[i] == 0 && request.current_velocity[i] == 0)
                
                    continue;
                


                bool case2_success = solve_case2(request.current_position[i], request.desired_position[i], request.current_velocity[i], request.desired_velocity[i],
                    v_max[i], a_max[i], out v1[i], out a1[i], out a3[i], out t1[i], out t2[i], out t3[i]);
                if (!case2_success)
                
                    bool case3_success = solve_case3(request.current_position[i], request.desired_position[i], request.current_velocity[i], request.desired_velocity[i],
                        a_max[i], out a1[i], out a3[i], out t1[i], out t3[i]);
                    t2[i] = 0;
                    v1[i] = 0;
                    if (!case3_success)
                    
                        throw new ArgumentException("Invalid trajectory request");
                    
                
            

            double t1_4 = t1.Max();
            double t2_4 = t2.Max();
            double t3_4 = t3.Max();

            double[] a1_4 = new double[joint_count];
            double[] a3_4 = new double[joint_count];
            double[] v1_4 = new double[joint_count];


            for (int i = 0; i < joint_count; i++)
            
                solve_case4(request.current_position[i], request.desired_position[i], request.current_velocity[i], request.desired_velocity[i],
                    t1_4, t2_4, t3_4, out v1_4[i], out a1_4[i], out a3_4[i]);
            

            var ret = new TrapezoidalJointTrajectoryGeneraterExec()
            
                joint_count = joint_count,
                t1 = t1_4,
                t2 = t2_4,
                t3 = t3_4,
                x1 = request.current_position,
                v1 = request.current_velocity,
                v2 = v1_4,
                v3 = request.desired_velocity,
                a1 = a1_4,
                a3 = a3_4,
                xf = request.desired_position
            ;

            return ret;
        

        internal static double pos_1(double a, double v, double x, double t)
        
            return (a != 0.0 ? 0.5 * a * Math.Pow(t, 2) : 0.0) + v * t + x;
        

        internal static double vel_1(double a, double v, double t)
        
            return (a != 0.0 ? a * t : 0) + v;
        

        internal static void pos(double a1, double a3, double x0, double v0, double t1, double t2, double t3, out double xf, out double vf)
        
            double v1_p = vel_1(a1, v0, t1);
            double v3_p = vel_1(a3, v1_p, t3);

            double x1_p = pos_1(a1, v0, x0, t1);
            double x2_p = pos_1(0, v1_p, x1_p, t2);
            double x3_p = pos_1(a3, v1_p, x2_p, t3);

            xf = x3_p;
            vf = v3_p;
        



        internal static bool solve_case2_sub(double x0, double xf, double v0, double v1, double vf, double a_max, out double a1, out double a3, out double t1, out double t2, out double t3)
        
            t1 = 0;
            a1 = 0;
            if (v1 != v0)
            
                a1 = a_max * Math.Sign(v1 - v0);
                t1 = (v1 - v0) / a1;
            
            t3 = 0;
            a3 = 0;
            if (vf != v1)
            
                a3 = a_max * Math.Sign(vf - v1);
                t3 = (vf - v1) / a3;
            

            pos(a1, a3, x0, v0, t1, 0, t3, out double xf_1, out double vf_1);

            double dx2 = xf - xf_1;

            t2 = dx2 / v1;

            return t2 >= 0;
        

        internal static bool solve_case2(double x0, double xf, double v0, double vf, double v_max, double a_max, out double v1, out double a1, out double a3, out double t1, out double t2, out double t3)
        
            if (solve_case2_sub(x0, xf, v0, v_max, vf, a_max, out a1, out a3, out t1, out t2, out t3))
            
                v1 = v_max;
                return true;
            

            if (solve_case2_sub(x0, xf, v0, -v_max, vf, a_max, out a1, out a3, out t1, out t2, out t3))
            
                v1 = -v_max;
                return true;
            
            v1 = 0;

            return false;
        

        internal static double solve_case3_sub1(double x0, double xf, double v0, double vf, double a1)
        
            return a1 * (xf - x0) + 0.5 * Math.Pow(v0, 2.0) + 0.5 * Math.Pow(vf, 2.0);
        

        internal static bool solve_case3(double x0, double xf, double v0, double vf, double a_max, out double a1, out double a3, out double t1, out double t3)
        
            double sub1 = solve_case3_sub1(x0, xf, v0, vf, a_max);
            if (sub1 >= 0)
            
                a1 = a_max;
                a3 = -a_max;
                t1 = (-v0 + Math.Sqrt(sub1)) / a1;
                t3 = (a1 * t1 + v0 - vf) / a1;
                if (t1 > 0 && t3 > 0)
                    return true;

                t1 = (-v0 - Math.Sqrt(sub1)) / a1;
                t3 = a1 * (a1 * t1 + v0 - vf) / a1;
                if (t1 > 0 && t3 > 0)
                    return true;
            

            sub1 = solve_case3_sub1(x0, xf, v0, vf, -a_max);
            if (sub1 >= 0)
            
                a1 = -a_max;
                a3 = a_max;
                t1 = (-v0 + Math.Sqrt(sub1)) / a1;
                t3 = (a1 * t1 + v0 - vf) / a1;
                if (t1 > 0 && t3 > 0)
                    return true;

                t1 = (-v0 - Math.Sqrt(sub1)) / a1;
                t3 = (a1 * t1 + v0 - vf) / a1;
                if (t1 > 0 && t3 > 0)
                    return true;
            
            t1 = 0;
            t3 = 0;
            a1 = 0;
            a3 = 0;
            return false;
        

        static void solve_case4(double x0, double xf, double v0, double vf, double t1, double t2, double t3, out double v1, out double a1, out double a3)
        
            double a1_den = (t1 * (t1 + 2 * t2 + t3));
            a1 = (a1_den != 0) ? (-2 * t1 * v0 - 2 * t2 * v0 - t3 * v0 - t3 * vf - 2 * x0 + 2 * xf) / a1_den : 0.0;
            v1 = a1 * t1 + v0;
            a3 = 0;
            if (t3 != 0)
            
                a3 = (-a1 * t1 - v0 + vf) / t3;
            
        

        public static TrapezoidalJointTrajectoryGeneraterExec InitializeVelExec(uint joint_count, JointTrajectoryLimits limits, JointTrajectoryVelocityRequest request)
        
            double[] a_max = (double[])limits.a_max.Clone();
            double[] v_max = (double[])limits.v_max.Clone();
            if (request.speed_ratio != 0.0)
            
                for (int i = 0; i < joint_count; i++)
                
                    a_max[i] = a_max[i] * request.speed_ratio;
                    v_max[i] = v_max[i] * request.speed_ratio;

                    double req_vel = request.desired_velocity[i] * request.speed_ratio;
                    if (req_vel < v_max[i])
                    
                        v_max[i] = req_vel;
                    
                    if (req_vel > v_max[i])
                    
                        throw new ArgumentException("req_vel must be less than or equal to v_max ");
                    
                
            
            else
            
                for (int i = 0; i < joint_count; i++)
                
                    if (request.desired_velocity[i] > v_max[i])
                    
                        throw new ArgumentException("req_vel must be less than or equal to v_max ");
                    

                    if (request.desired_velocity[i] < v_max[i])
                    
                        v_max[i] = request.desired_velocity[i];
                    
                
            

            double[] t1 = new double[joint_count];
            double[] t2 = new double[joint_count];
            double[] t3 = new double[joint_count];

            double[] v1 = new double[joint_count];
            double[] a1 = new double[joint_count];
            double[] a3 = new double[joint_count];

            for (int i = 0; i < joint_count; i++)
            
                if (request.desired_velocity[i] == 0 && request.current_velocity[i] == 0)
                
                    continue;
                


                solve_case5(request.current_velocity[i], request.desired_velocity[i], 0, request.timeout,
                     a_max[i], out v1[i], out a1[i], out a3[i], out t1[i], out t2[i], out t3[i]);
                
            

            double t1_4 = t1.Max();
            double t2_4 = t2.Max();
            double t3_4 = t3.Max();

            double[] a1_4 = new double[joint_count];
            double[] a3_4 = new double[joint_count];
            

            for (int i = 0; i < joint_count; i++)
            
                solve_case6(request.current_velocity[i], v1[i], 0,
                    t1_4, t2_4, t3_4,  out a1_4[i], out a3_4[i]);
            

            var ret = new TrapezoidalJointTrajectoryGeneraterExec()
            
                joint_count = joint_count,
                t1 = t1_4,
                t2 = t2_4,
                t3 = t3_4,
                x1 = request.current_position,
                v1 = request.current_velocity,
                v2 = v1,
                v3 = new double[joint_count],
                a1 = a1_4,
                a3 = a3_4,
                //xf = request.desired_position
            ;

            return ret;
        

        internal static bool solve_case5(double v0, double v1, double vf, double timeout, double a_max, out double v1_res, out double a1, out double a3, out double t1, out double t2, out double t3)
        
            t1 = 0;
            a1 = 0;
            if (v1 != v0)
            
                a1 = a_max * Math.Sign(v1 - v0);
                t1 = (v1 - v0) / a1;
            

            if (t1 > timeout)
            
                v1 = a1 * timeout;
                t1 = timeout;
            

            t3 = 0;
            a3 = 0;
            if (vf != v1)
            
                a3 = a_max * Math.Sign(vf - v1);
                t3 = (vf - v1) / a3;
            

            t2 = timeout - t1;

            v1_res = v1;

            return true;
        

        static void solve_case6(double v0, double v1, double vf, double t1, double t2, double t3, out double a1, out double a3)
        
            double a1_den = t1;
            a1 = (a1_den != 0) ? (v1-v0) / a1_den : 0.0;
            a3 = 0;
            if (t3 != 0)
            
                a3 = (-a1 * t1 - v0 + vf) / t3;
            
        


    


    class TrapezoidalJointTrajectoryGeneraterExec
    
        public uint joint_count;
        public double t1;
        public double t2;
        public double t3;
        public double[] x1;
        public double[] v1;
        public double[] v2;
        public double[] v3;
        public double[] a1;
        public double[] a3;

        protected double[] x2;
        protected double[] x3;
        public double[] xf;


        internal static double pos(double a, double v, double x, double t)
        
            return (a!=0.0 ? 0.5 * a * Math.Pow(t, 2) : 0.0) + v * t + x;
        

        internal static double vel(double a, double v, double t)
        
            return (a!=0.0 ? a * t : 0) + v;
        

        internal static void pos(uint n, double[] a, double[] v, double[] x, double t, ref double[] o)
        
            for (int i=0; i<n; i++)
            
                if (a != null)
                
                    o[i] = pos(a[i], v[i], x[i], t);
                
                else
                
                    o[i] = pos(0.0, v[i], x[i], t);
                
            
        

        internal static void vel(uint n, double[] a, double[] v, double t, ref double[] o)
        
            for (uint i = 0; i < n; i++)
            
                if (a != null)
                
                    o[i] = vel(a[i], v[i], t);
                
                else
                
                    o[i] = vel(0.0, v[i], t);
                
            
        

        public bool CalcAtTime(double t, ref double[] x, ref double[] v)
        
            if (x2 == null)
            
                x2 = new double[joint_count];
                pos(joint_count, a1, v1, x1, t1, ref x2);
            

            if (x3 == null)
            
                x3 = new double[joint_count];
                pos(joint_count, null, v2, x2, t2, ref x3);
            

            if (xf == null)
            
                xf = new double[joint_count];
                pos(joint_count, a3, v2, x3, t3, ref xf);
            

            if (t < 0)
            
                return false;
            

            if (t < t1)
                           
                pos(joint_count, a1, v1, x1, t, ref x);
                vel(joint_count, a1, v1, t, ref v);
                return true;
            

            if (t < t2+t1)
            
                pos(joint_count, null, v2, x2, t - t1, ref x);
                vel(joint_count, null, v2, t - t1, ref v);
                return true;
            

            if (t < t3+t2+t1)
            
                pos(joint_count, a3, v2, x3, t - t1 - t2, ref x);
                vel(joint_count, a3, v2, t - t1 - t2, ref v);
                return true;
            

            pos(joint_count, null, v3, xf, t - t1 - t2 - t3, ref x);
            vel(joint_count, null, v3, t - t1 - t2 - t3, ref v);
            return true;
        

        public double t_final => t1 + t2 + t3;
    

using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using System;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using BepuUtilities.Memory;
using static BepuUtilities.GatherScatter;
using Demos.Demos.Characters;
using BepuUtilities.Collections;
using DemoRenderer.UI;
using OpenTK.Input;
using System.Collections.Generic;
using Com.Nelalen.GameObject;

namespace Demos.Demos
{
    /// <summary>
    /// Convenience structure that wraps a CharacterController reference and its associated body.
    /// </summary>
    /// <remarks>
    /// This should be treated as an example- nothing here is intended to suggest how you *must* handle characters. 
    /// On the contrary, this does some fairly inefficient stuff if you're dealing with hundreds of characters in a predictable way.
    /// It's just a fairly convenient interface for demos usage.
    /// </remarks>


    public class MolDemo : Demo
    {
        bool characterActive;
        CharacterControllers characs;
        private BufferPool bufferPool;
        private SimpleThreadDispatcher threadDispatcher;
        private long id = -1;
        //private Dictionary<long, Map> maps = new Dictionary<long, Map>();
        // <charId,Character>
        private SortedDictionary<int, Character> characters = new SortedDictionary<int, Character>();
        private PlayerCharacter character;
        // <handleId, Character(that walk)>
        private SortedDictionary<int, Character> walkCharacters = new SortedDictionary<int, Character>();
        private List<int> walkRemoveCharacters = new List<int>();
        // <handleId, Character>
        private SortedDictionary<int, Unit> handleUnits = new SortedDictionary<int, Unit>();
        //private ContactEvents<IContactEventHandler> contactEvent = new ContactEvents<IContactEventHandler>();

        private bool isStop;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(20, 10, 20);
            camera.Yaw = MathF.PI;
            camera.Pitch = 0;
            bufferPool = new BufferPool();
            var collider = new BodyProperty<Collider>();
            //characters = new CharacterControllers(BufferPool);
            characs = new CharacterControllers(BufferPool);
            //Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            Simulation = Simulation.Create(BufferPool, new MolCallbacks(characs, collider ), new DemoPoseIntegratorCallbacks(new System.Numerics.Vector3(0, -10, 0)));
            character = new PlayerCharacter(1, 1, "noname", 1, this, new Vector3(0f, 3f, 0f));
            AddPlayerCharacter(character);
            //Prevent the character from falling into the void.
            //Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new System.Numerics.Vector3(0, -2f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.4f)));

        }


        public override void Update(Window window, Camera camera, Input input, float dt)
        {

            const float simulationDt = 1 / 60f;
            if (characterActive)
            {
                character.CharacterInputt.UpdateCharacterGoals(new Vector2(1,2), simulationDt, input, camera);
            }
            
            //Using a fixed time per update to match the demos simulation update rate.
#if DEBUG
            Console.WriteLine(character.Position);
#endif
            base.Update(window, camera, input, dt);
        }
        internal void AddCharacter(Character character)
        {
            var characterShape = character.Shape();
            characterShape.ComputeInertia(1, out var characterInertia);

            int handle = Simulation.Bodies.Add(BodyDescription.CreateDynamic(character.GetStartPosition(), characterInertia, new CollidableDescription(Simulation.Shapes.Add(characterShape), 0.1f), new BodyActivityDescription(0.01f)));

            handleUnits.Add(handle, character);
            character.SetBodyHandle(handle);
            character.PeriodicAABB();
            character.SetCharacterInput(bufferPool, handle, Simulation);
            //character.SetBodyReference(Simulation.Bodies.GetBodyReference(handle));
        }

        internal void AddPlayerCharacter(PlayerCharacter character)
        {
            characterActive = true;
            int charId = character.GetClientId();
            if (characters.ContainsKey(charId))
            {
                characters.Remove(charId);
            }
            characters.Add(charId, character);
            AddCharacter(character);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            float textHeight = 16;
            var position = new Vector2(32, renderer.Surface.Resolution.Y - textHeight * 9);
            renderer.TextBatcher.Write(text.Clear().Append("Toggle character: C"), position, textHeight, new Vector3(1), font);
            position.Y += textHeight * 1.2f;
            character.CharacterInputt.RenderControls(position, textHeight, renderer.TextBatcher, text, font);
            character.CharacterInputt.UpdateCameraPosition(camera);
            base.Render(renderer, camera, input, text, font);
        }

        internal Unit GetHandleUnit(int handle)
        {
            handleUnits.TryGetValue(handle, out Unit value);
            return value;
        }
    }

    struct MolCallbacks : INarrowPhaseCallbacks
    {
        public CharacterControllers Characters;
        public BodyProperty<Collider> Collider;

        public MolCallbacks(CharacterControllers characters, BodyProperty<Collider> Collider)
        {
            this.Characters = characters;
            this.Collider = Collider; 
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial = new PairMaterialProperties { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            GetMaterial(out pairMaterial);
            Characters.TryReportContacts(pair, ref *manifold, workerIndex, ref pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            GetMaterial(out pairMaterial);
            Characters.TryReportContacts(pair, ref *manifold, workerIndex, ref pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
            Characters.Dispose();
        }

        public void Initialize(Simulation simulation)
        {
            Characters.Initialize(simulation);
        }
    }

    public struct DemoPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity;
        public float LinearDamping;
        public float AngularDamping;
        Vector3 gravityDt;
        float linearDampingDt;
        float angularDampingDt;

        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public DemoPoseIntegratorCallbacks(Vector3 gravity, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            Gravity = gravity;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
        }

        public void PrepareForIntegration(float dt)
        {
            //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
            gravityDt = Gravity * dt;
            //Since this doesn't use per-body damping, we can precalculate everything.
            linearDampingDt = MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt);
            angularDampingDt = MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity)
        {
            //Note that we avoid accelerating kinematics. Kinematics are any body with an inverse mass of zero (so a mass of ~infinity). No force can move them.
            if (localInertia.InverseMass > 0)
            {
                velocity.Linear = (velocity.Linear + gravityDt) * linearDampingDt;
                velocity.Angular = velocity.Angular * angularDampingDt;
            }
            //Implementation sidenote: Why aren't kinematics all bundled together separately from dynamics to avoid this per-body condition?
            //Because kinematics can have a velocity- that is what distinguishes them from a static object. The solver must read velocities of all bodies involved in a constraint.
            //Under ideal conditions, those bodies will be near in memory to increase the chances of a cache hit. If kinematics are separately bundled, the the number of cache
            //misses necessarily increases. Slowing down the solver in order to speed up the pose integrator is a really, really bad trade, especially when the benefit is a few ALU ops.

            //Note that you CAN technically modify the pose in IntegrateVelocity. The PoseIntegrator has already integrated the previous velocity into the position, but you can modify it again
            //if you really wanted to.
            //This is also a handy spot to implement things like position dependent gravity or per-body damping.
        }

    }
}



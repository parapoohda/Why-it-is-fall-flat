using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using BepuPhysics;
using Quaternion = BepuUtilities.Quaternion;
using BepuPhysics.Collidables;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;
using DemoRenderer;
using Demos.Demos;

namespace Com.Nelalen.GameObject
{
    
    unsafe struct HitHandler : BepuUtilities.IBreakableForEach<CollidableReference>
    {
        public Character character;
        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public bool LoopBody(CollidableReference collidable)
        {
            character.AddCharTolist(collidable.Handle);
            return true;
        }
    }

    internal class Character : Unit
    {
        private CharacterInput characterInput;
        internal CharacterInput CharacterInputt => characterInput;
        private float walkSpeed = 1f;
        private float runSpeed = 2f;
        private bool isRun;
        private bool isCalculateVelocityYet = true;
        internal bool IsCalculateVelocityYet => isCalculateVelocityYet;
        private MolDemo map;
        internal MolDemo GetMap() { return map; }
        private System.Numerics.Vector3 sizeBB = new System.Numerics.Vector3(16, 16, 16);
        private System.Numerics.Vector3 target = new System.Numerics.Vector3();
        internal System.Numerics.Vector3 Target => target;
        internal enum BepuTypeId { Sphere, Capsule }
        internal System.Numerics.Vector3 Position => new BepuPhysics.BodyReference(collider.bodyHandle, map.Simulation.Bodies).Pose.Position;
        private BepuTypeId shapeTypeId = BepuTypeId.Capsule;
        private System.Numerics.Vector2 velocity;
        private Demos.Demos.Characters.CharacterControllers characterControllers;
        private System.Numerics.Vector3 viewDirection;
        internal System.Numerics.Vector3 ViewDirection => viewDirection;

        internal System.Numerics.Vector2 Velocity => velocity;

        internal Character(int unitId, string name, MolDemo map, System.Numerics.Vector3 startPosition) : base(unitId, name, startPosition)
        {
            this.map = map;
            collider.isPassThrough = false;
            collider.type = Collider.Type.Characer;
        }

        internal float CalculateDistant()
        {
            var distantVector3 = target - Position;
            var distantVector2 = new System.Numerics.Vector2(distantVector3.X, distantVector3.Z);

            return MathF.Sqrt(distantVector2.LengthSquared());
        }


        internal void CalculateVelocity()
        {
            float moveSpeed;
            if (isRun) {
                moveSpeed = walkSpeed;
            } else {
                moveSpeed = runSpeed;
            }
            velocity.X = target.X - Position.X;
            velocity.Y = target.Z - Position.Z;
            velocity /= velocity.LengthSquared();
#if DEBUG
            Console.WriteLine($"charac velocity : {velocity}");
#endif
            
            isCalculateVelocityYet = true;
        }

        internal void SetCharacterInput(BepuUtilities.Memory.BufferPool bufferpool, int bodyHandle, Simulation simulation)
        {
            characterControllers = new Demos.Demos.Characters.CharacterControllers(bufferpool);
            collider.bodyHandle = bodyHandle;
            
            characterInput = new CharacterInput(characterControllers, bodyHandle, simulation, this.Position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
        }

        private int CalculateMagnitudeI;
        internal float CalculateMagnitude()
        {
            if (CalculateMagnitudeI++ % 100 == 0)
            {
            }
            return (this.target - Position).LengthSquared();
        }


        internal void PeriodicAABB()
        {
            var hitHandler = new HitHandler { character = this };
            System.Numerics.Vector3 minBB = Position - (sizeBB/2);
            System.Numerics.Vector3 maxBB = Position + (sizeBB/2);
            var box = new BepuUtilities.BoundingBox(minBB, maxBB);
            map.Simulation.BroadPhase.GetOverlaps(box, ref hitHandler);
        }


        internal Capsule Shape() {
            return new Capsule(.3f,1.5f);
        }

        internal void AddCharTolist(int handle)
        {
            //TODO TO DO Here character type is collider
            var character = map.GetHandleUnit(handle);
            if (character.collider.type == Collider.Type.PlayerCharacer) {
                PlayerCharacter charac = (PlayerCharacter)character;
            }
        }

        internal BepuTypeId ShapeTypeId => shapeTypeId;
        internal struct CharacterInput
        {
            int bodyHandle;
            Demos.Demos.Characters.CharacterControllers characters;
            float speed;
            Capsule shape;
            Simulation Simulation;

            static Key MoveForward = Key.W;
            static Key MoveBackward = Key.S;
            static Key MoveRight = Key.D;
            static Key MoveLeft = Key.A;
            static Key Sprint = Key.LShift;
            static Key Jump = Key.Space;
            static Key JumpAlternate = Key.BackSpace; //I have a weird keyboard.
            internal int BodyHandle { get { return bodyHandle; } }

            internal CharacterInput(Demos.Demos.Characters.CharacterControllers characters, int bodyHandle, Simulation simulation, System.Numerics.Vector3 initialPosition, Capsule shape,
                float speculativeMargin, float mass, float maximumHorizontalForce, float maximumVerticalGlueForce,
                float jumpVelocity, float speed, float maximumSlope = MathF.PI * 0.25f)
            {
                this.bodyHandle = bodyHandle;
                this.Simulation = simulation;
                this.characters = characters;

                ref var character = ref characters.AllocateCharacter(bodyHandle);
                character.LocalUp = new System.Numerics.Vector3(0, 1, 0);
                character.CosMaximumSlope = MathF.Cos(maximumSlope);
                character.JumpVelocity = jumpVelocity;
                character.MaximumVerticalForce = maximumVerticalGlueForce;
                character.MaximumHorizontalForce = maximumHorizontalForce;
                character.MinimumSupportDepth = shape.Radius * -0.01f;
                character.MinimumSupportContinuationDepth = -speculativeMargin;
                this.speed = speed;
                this.shape = shape;
            }
            public void UpdateCharacterGoals(System.Numerics.Vector2 movementDirection, float simulationTimestepDuration, Input input, Camera camera)
            {
                
                var movementDirectionLengthSquared = movementDirection.LengthSquared();
                if (movementDirectionLengthSquared > 0)
                {
                    movementDirection /= MathF.Sqrt(movementDirectionLengthSquared);
                }

                ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
                var characterBody = new BodyReference(bodyHandle, Simulation.Bodies);
                var effectiveSpeed = 1f;
                var newTargetVelocity = movementDirection * effectiveSpeed;
                var viewDirection = new Vector3(-1f, 0f, 0f);
                //Modifying the character's raw data does not automatically wake the character up, so we do so explicitly if necessary.
                //If you don't explicitly wake the character up, it won't respond to the changed motion goals.
                //(You can also specify a negative deactivation threshold in the BodyActivityDescription to prevent the character from sleeping at all.)
                
                Simulation.Awakener.AwakenBody(character.BodyHandle);
                
                character.TargetVelocity = newTargetVelocity;
                character.ViewDirection = viewDirection;

                //The character's motion constraints aren't active while the character is in the air, so if we want air control, we'll need to apply it ourselves.
                //(You could also modify the constraints to do this, but the robustness of solved constraints tends to be a lot less important for air control.)
                //There isn't any one 'correct' way to implement air control- it's a nonphysical gameplay thing, and this is just one way to do it.
                //Note that this permits accelerating along a particular direction, and never attempts to slow down the character.
                //This allows some movement quirks common in some game character controllers.
                //Consider what happens if, starting from a standstill, you accelerate fully along X, then along Z- your full velocity magnitude will be sqrt(2) * maximumAirSpeed.
                //Feel free to try alternative implementations. Again, there is no one correct approach.
                if (!character.Supported && movementDirectionLengthSquared > 0)
                {
                    Quaternion.Transform(character.LocalUp, characterBody.Pose.Orientation, out var characterUp);
                    var characterRight = Vector3.Cross(character.ViewDirection, characterUp);
                    var rightLengthSquared = characterRight.LengthSquared();
                    if (rightLengthSquared > 1e-10f)
                    {
                        characterRight /= MathF.Sqrt(rightLengthSquared);
                        var characterForward = Vector3.Cross(characterUp, characterRight);
                        var worldMovementDirection = characterRight * movementDirection.X + characterForward * movementDirection.Y;
                        var currentVelocity = Vector3.Dot(characterBody.Velocity.Linear, worldMovementDirection);
                        //We'll arbitrarily set air control to be a fraction of supported movement's speed/force.
                        const float airControlForceScale = .2f;
                        const float airControlSpeedScale = .2f;
                        var airAccelerationDt = characterBody.LocalInertia.InverseMass * character.MaximumHorizontalForce * airControlForceScale * simulationTimestepDuration;
                        var maximumAirSpeed = effectiveSpeed * airControlSpeedScale;
                        var targetVelocity = MathF.Min(currentVelocity + airAccelerationDt, maximumAirSpeed);
                        //While we shouldn't allow the character to continue accelerating in the air indefinitely, trying to move in a given direction should never slow us down in that direction.
                        var velocityChangeAlongMovementDirection = MathF.Max(0, targetVelocity - currentVelocity);
                        characterBody.Velocity.Linear += worldMovementDirection * velocityChangeAlongMovementDirection;
                        Debug.Assert(characterBody.Awake, "Velocity changes don't automatically update objects; the character should have already been woken up before applying air control.");
                    }
                }
            }
            void RenderControl(ref Vector2 position, float textHeight, string controlName, string controlValue, TextBuilder text, TextBatcher textBatcher, Font font)
            {
                text.Clear().Append(controlName).Append(": ").Append(controlValue);
                textBatcher.Write(text, position, textHeight, new Vector3(1), font);
                position.Y += textHeight * 1.1f;
            }
            public void RenderControls(Vector2 position, float textHeight, TextBatcher textBatcher, TextBuilder text, Font font)
            {
                RenderControl(ref position, textHeight, nameof(MoveForward), MoveForward.ToString(), text, textBatcher, font);
                RenderControl(ref position, textHeight, nameof(MoveBackward), MoveBackward.ToString(), text, textBatcher, font);
                RenderControl(ref position, textHeight, nameof(MoveRight), MoveRight.ToString(), text, textBatcher, font);
                RenderControl(ref position, textHeight, nameof(MoveLeft), MoveLeft.ToString(), text, textBatcher, font);
                RenderControl(ref position, textHeight, nameof(Sprint), Sprint.ToString(), text, textBatcher, font);
                RenderControl(ref position, textHeight, nameof(Jump), Jump.ToString(), text, textBatcher, font);
            }
            public void UpdateCameraPosition(Camera camera, float cameraBackwardOffsetScale = 4)
            {
                //We'll override the demo harness's camera control by attaching the camera to the character controller body.
                ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
                var characterBody = new BodyReference(bodyHandle, Simulation.Bodies);
                //Use a simple sorta-neck model so that when the camera looks down, the center of the screen sees past the character.
                //Makes mouselocked ray picking easier.
                camera.Position = characterBody.Pose.Position + new Vector3(0, shape.HalfLength, 0) +
                    camera.Up * (shape.Radius * 1.2f) -
                    camera.Forward * (shape.HalfLength + shape.Radius) * cameraBackwardOffsetScale;
            }

            /// <summary>
            /// Removes the character's body from the simulation and the character from the associated characters set.
            /// </summary>
            public void Dispose()
            {
                Simulation.Shapes.Remove(new BodyReference(bodyHandle, Simulation.Bodies).Collidable.Shape);
                Simulation.Bodies.Remove(bodyHandle);
                characters.RemoveCharacterByBodyHandle(bodyHandle);
            }
        }

        
    }
}


using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Com.Nelalen.GameObject
{
    internal class Unit 
    {
        internal Collider collider;
        private string name;
        private int unitId;
        private System.Numerics.Vector3 startPosition = new System.Numerics.Vector3(0f, 1f, 0f);
        internal Unit(int unitId, string name, System.Numerics.Vector3 startPosition) {
            collider.type = Collider.Type.Unit;
            this.name = name;
            this.unitId = unitId;
            //TODO to do have to make start position relevant with last position
            //this.startPosition = startPosition;
            this.startPosition.Y = 3;
        }
        
         internal System.Numerics.Vector3 GetStartPosition()
        {
            return this.startPosition;
        }
        
        internal int GetUnitId() {
            return unitId;
        }
        internal string GetName() {
            return name;
        }

        public void SetBodyHandle(int bodyHandle) {
            //Console.WriteLine("bodyHandle: ", bodyHandle);
            collider.bodyHandle = bodyHandle;
        }
    }
}

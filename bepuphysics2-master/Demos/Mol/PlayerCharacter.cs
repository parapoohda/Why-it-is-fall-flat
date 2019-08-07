
using Com.Nelalen.GameObject;
using Demos.Demos;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace Com.Nelalen.GameObject
{
    internal class PlayerCharacter : Character
    {
        private int charId;
        
        private int characterId { get { return charId; } set { charId = value; } }
        private int clientId;

        internal PlayerCharacter(int characterId, int clientId, string name, int unitId, MolDemo map, System.Numerics.Vector3 startPosition) : base(unitId, name, map, startPosition)
        {
            collider.type = Collider.Type.PlayerCharacer;
            this.charId = characterId;
            this.clientId = clientId;
        }


        internal int GetClientId() {
            return clientId;
        }

        internal int GetCharId() {
            return charId;
        }



        /*internal void CharacterMove(Vector3 target, CharacterMoveType characterMoveType, float movementSpeed)
        {
            gate.CharacterMove(target, characterMoveType, movementSpeed);
        }*/
    }
}

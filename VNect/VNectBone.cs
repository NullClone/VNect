using UnityEngine;

namespace VNect
{
    public class VNectBone
    {
        public Transform Transform;
        public VNectBone Child;
        public VNectBone Parent;

        public Vector3 Forward;
        public Vector3 Position3D;

        public Quaternion InitRotation;
        public Quaternion InverseRotation;
        public Quaternion Inverse;
    }
}
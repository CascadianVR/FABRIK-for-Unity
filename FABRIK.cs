using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// Referenced this paper for algorithm: http://www.andreasaristidou.com/publications/papers/FABRIK.pdf

namespace AIConversation
{
    public class FABRIK : MonoBehaviour
    {
        public Transform target;
        public float tolerance = 0.1f;
        public Transform[] joints;
        
        private Vector3[] jointPositions;
        private float[] jointLengths;
        
        private void Start()
        {
            Setup();
        }

        private void Setup()
        {
            jointLengths = new float[joints.Length - 1];
            jointPositions = new Vector3[joints.Length];

            for (int i = 0; i < jointLengths.Length; i++)
            {
                jointLengths[i] = Vector3.Distance(joints[i + 1].position, joints[i].position);
            }
        }

        private void Update()
        {
            SolveFABRIK();
        }

        private void SolveFABRIK()
        {
            float totalDistance = Vector3.Distance(joints[0].position, target.position);
            float totalLength = jointLengths.Sum();
            
            if (totalDistance > totalLength) // Check to see if target is reachable.
            {
                // If not reachable, keep root in place and stretch to max length.
                
                for (int i = 0; i < joints.Length-1; i++)
                {
                    Vector3 tpos = target.position;
                    float r = Vector3.Distance(joints[i].position, tpos);
                    float lambda = jointLengths[i] / r;

                    joints[i + 1].position = (1f - lambda) * joints[i].position + lambda * tpos;
                }
            }
            else
            {
                // If reachable, continue solving FABRIK algorithm

                Vector3 rootPos = joints[0].position;
                float diff = Vector3.Distance(joints[^1].position, target.position);

                while (diff > tolerance)
                {
                    // Forward Stage
                    joints[^1].position = target.position;
                    for (int i = joints.Length - 2; i >= 0; i--)
                    {
                        float r = Vector3.Distance(joints[i].position, joints[i+1].position);
                        float lambda = jointLengths[i] / r;

                        joints[i].position = (1f - lambda) * joints[i+1].position + lambda * joints[i].position;
                    }
                    
                    // Backward Stage
                    joints[0].position = rootPos;
                    for (int i = 0; i < joints.Length - 1; i++)
                    {
                        float r = Vector3.Distance(joints[i].position, joints[i+1].position);
                        float lambda = jointLengths[i] / r;

                        joints[i + 1].position = (1f - lambda) * joints[i].position + lambda * joints[i + 1].position;
                    }

                    diff = Vector3.Distance(joints[^1].position, target.position);
                }
            }
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Position Constraint ,will keep target joint's position.
/// </summary>
public class MMPositionConstraint : MonoBehaviour
{
    // Start is called before the first frame update
    public const float MIN__TOLERENCE = 0.001f;

    /// <summary>
    /// the constraint error's weight;
    /// </summary>
    [Range(0, 1)]
    public float weight;
    /// <summary>
    /// the constraint error's tolerance;
    /// </summary>
    [Range(MIN__TOLERENCE, 1f)]
    public float tolerance;
    /// <summary>
    /// Constraint target
    /// </summary>
    public Transform targetJointTransform;
}

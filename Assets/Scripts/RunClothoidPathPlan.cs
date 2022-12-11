using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using MathNet.Numerics;
using Accord.Math;
using Utils;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class RunClothoidPathPlan : MonoBehaviour
{

    [SerializeField] private UnityEngine.Vector3 StartPosition;
    [SerializeField] private UnityEngine.Vector3 GoalPosition;
    [SerializeField] private InputField OrientationIntervalnput;
    [SerializeField] private InputField OrientationStepsInput;
    [SerializeField] private InputField NumberOfTrajsInput;

    [SerializeField] private GameObject CursorPrefab;
    private GameObject Cursor;
    [SerializeField] private UnityEngine.Vector3 MouseWorldPos;
    [SerializeField] private Camera Camera;
    private enum Modes
    {
        _,
        SetStartMode,
        SetGoalMode
    }
    [SerializeField] private Modes Mode;

    void Start()
    {
        Cursor = Instantiate(CursorPrefab);
        Mode = Modes._;

        OrientationIntervalnput.text = "1.57";
        OrientationStepsInput.text = "25";
        NumberOfTrajsInput.text = "50";
    }

    void Update()
    {
        OnMouseClick();
        UpdateMousePosition();
        UpdateCursor();
    }

    private void ClothoidPath(UnityEngine.Vector3 startPosition, UnityEngine.Vector3 goalPosition)
    {
        // Set Inputs.
        var start_point = new Point(startPosition.x, startPosition.z);
        var start_orientation_list = new List<double> { 0.0 };
        var goal_point = new Point(goalPosition.x, goalPosition.z);
        var goal_orientation_list = Vector.Interval((-float.Parse(OrientationIntervalnput.text)), (float.Parse(OrientationIntervalnput.text)), (int.Parse(OrientationStepsInput.text))).ToList();
        var num_path_points = int.Parse(NumberOfTrajsInput.text);
        // Generate Plan.
        ClothoidPathPlanner clothoidPathPlanner = new ClothoidPathPlanner();
        var clothoid_paths = clothoidPathPlanner.generate_clothoid_paths(
            start_point, start_orientation_list,
            goal_point, goal_orientation_list,
            num_path_points);

        // Visualise Trajectories.
        float x = 0;
        float z = 0;
        UnityEngine.Vector3 pos = UnityEngine.Vector3.zero;
        for (int i=0; i < clothoid_paths.Count; i++)
        {
            var clothoid_path = clothoid_paths[i];

            GameObject lineGO = new GameObject("Line");
            LineRenderer lineRenderer = lineGO.AddComponent<LineRenderer>();
            lineRenderer.positionCount = clothoid_path.Count;
            lineRenderer.widthMultiplier = 0.1f;
            List<UnityEngine.Vector3> clothoidPathPosList = new List<UnityEngine.Vector3>();

            for (int j=0; j < clothoid_path.Count; j++)
            {
                x = clothoid_path[j].x;
                z = clothoid_path[j].y;
                pos.Set(x, 0.1f, z);
                clothoidPathPosList.Add(pos);
            }
            lineRenderer.SetPositions(clothoidPathPosList.ToArray());
        }
    }


    private void OnMouseClick()
    {
        if (Input.GetMouseButtonUp(0) && !EventSystem.current.IsPointerOverGameObject()) // Left click while ignoring scene object!
        {
            if (Mode == Modes.SetStartMode)
            {
                StartPosition = MouseWorldPos;
            }
            else if (Mode == Modes.SetGoalMode)
            {
                GoalPosition = MouseWorldPos;
            }
            if (Mode != Modes._) { Mode = Modes._; }
        }
    }

    private void UpdateMousePosition()
    {
        UnityEngine.Vector3 mousePos = new UnityEngine.Vector3(Input.mousePosition.x, Input.mousePosition.y, 0.1f);
        Ray ray = Camera.ScreenPointToRay(mousePos);
        if (Physics.Raycast(ray, out RaycastHit hit, 1100f))
        {
            MouseWorldPos = hit.point;
            DetermineHoveredObject(hit);
            return;
        }

        MouseWorldPos = Camera.ScreenToWorldPoint(mousePos);
    }

    private void DetermineHoveredObject(RaycastHit raycastHit)
    {
        Collider currentCollider = raycastHit.collider;
        GameObject currentObjectAtMousePos;

        if (currentCollider.attachedRigidbody != null)
        {
            currentObjectAtMousePos = currentCollider.attachedRigidbody.gameObject;
        }
        else
        {
            currentObjectAtMousePos = currentCollider.gameObject;
        }
    }

    private void UpdateCursor()
    {
        if (Cursor != null)
        {
            UnityEngine.Vector3 mousePos = new UnityEngine.Vector3(MouseWorldPos.x, 0.5f, MouseWorldPos.z);

            Cursor.transform.position = mousePos;

            if (Mode == Modes.SetStartMode)
            {
                Cursor.GetComponent<Light>().color = Color.green;
            }
            else if (Mode == Modes.SetGoalMode)
            {
                Cursor.GetComponent<Light>().color = Color.red;
            }
            else
            {
                Cursor.GetComponent<Light>().color = Color.gray;
            }
        }
    }


    public void OnSelectStartPosition()
    {
        Mode = Modes.SetStartMode;
    }

    public void OnSelectGoalPosition()
    {
        Mode = Modes.SetGoalMode;
    }

    public void OnRunSearch()
    {
        ClothoidPath(StartPosition, GoalPosition);
    }

}

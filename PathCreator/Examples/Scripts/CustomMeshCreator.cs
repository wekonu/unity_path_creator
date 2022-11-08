using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PathCreation.Examples
{
	public class CustomMeshCreator : PathSceneTool
	{
		private Mesh meshCache;
		MeshFilter meshFilter;
		MeshRenderer meshRenderer;

		[Header("Mesh Setting")]
		public Mesh customMesh;

		[Header("Material settings")]
		public Material customMaterial;
		public float textureTiling = 1;

		[SerializeField, HideInInspector]
		GameObject meshHolder;

		[SerializeField, HideInInspector]
		private List<float> distinctVertexPositions;
		[SerializeField, HideInInspector]
		private List<Vector3> verticesOffset;

		[SerializeField, HideInInspector]
		private List<Vector3> tangentsAveraged;
		[SerializeField, HideInInspector]
		private List<Vector3> tangents;
		[SerializeField, HideInInspector]
		private List<Vector3> points;
		[SerializeField, HideInInspector]
		private List<Vector3> binorms;
		[SerializeField, HideInInspector]
		private List<Vector3> normals;

		[SerializeField, HideInInspector]
		public int VertCount;

		[SerializeField]
		public ExtrusionAxis extrusionAxis = ExtrusionAxis.X;

		//Measurements of the mesh to be extruded
		[SerializeField, HideInInspector]
		private float MeshWidth;
		[SerializeField, HideInInspector]
		private float MeshLength;
		[SerializeField, HideInInspector]
		private float MeshHeight;

		//Measurements of the mesh collider
		[SerializeField, HideInInspector]
		public float CustomWidthOfCollider;
		[SerializeField, HideInInspector]
		public float CustomHeightOfCollider;

		//Enum to keep track of whether the mesh is extruded along the Z or X axis. 
		public enum ExtrusionAxis
		{
			Z,
			X
		}

		protected override void PathUpdated()
		{
			if (pathCreator != null && customMesh != null && customMaterial != null)
			{
				AssignMeshComponents();
				AssignMaterials();
				UpdateNewMesh();
			}
		}

		public void UpdateNewMesh()
		{
			if (customMesh == null)
			{
				Debug.LogError("No mesh chosen");
				return;
			}

			GetNewMeshData();
			UpdateMesh();

			//Get the measurements of the mesh
			//Get each distinct distance in the mesh along the extrusion axis 
			void GetNewMeshData()
			{

				//Find the minimum value of the mesh along the extrusion axis so that the mesh starts on 0 in that direction
				float minMeshValue = float.MaxValue;
				verticesOffset = new List<Vector3>();
				distinctVertexPositions = new List<float>();

				float widthOfMeshMinValue = float.MaxValue;
				float widthOfMeshMaxValue = float.MinValue;

				float heightOfMeshMinValue = float.MaxValue;
				float heightOfMeshMaxValue = float.MinValue;

				//Determine the width, height and length of mesh
				foreach (var vert in customMesh.vertices)
				{
					Vector3 p = vert;
					float vertexValue = 0;
					float vertexValueWidthAxis = 0;

					if (extrusionAxis.Equals(ExtrusionAxis.Z))
					{
						vertexValue = p.z;
						vertexValueWidthAxis = p.x;
					}
					else if (extrusionAxis.Equals(ExtrusionAxis.X))
					{
						vertexValue = p.x;
						vertexValueWidthAxis = p.z;
					}
					minMeshValue = Mathf.Min(minMeshValue, vertexValue); //for each vertex check if it is smaller then previously obtained value

					widthOfMeshMinValue = Mathf.Min(widthOfMeshMinValue, vertexValueWidthAxis);
					widthOfMeshMaxValue = Mathf.Max(widthOfMeshMaxValue, vertexValueWidthAxis);

					heightOfMeshMinValue = Mathf.Min(heightOfMeshMinValue, p.y);
					heightOfMeshMaxValue = Mathf.Max(heightOfMeshMaxValue, p.y);
				}

				//Determine all distinct vertex distances in the mesh along the extrusion axis
				//Offset the mesh vertices so they have their first position at 0 in the extrusion axis
				foreach (var vert in customMesh.vertices)
				{
					float offsetValue = 0;
					Vector3 vertOffset = vert;

					if (extrusionAxis.Equals(ExtrusionAxis.Z))
					{
						offsetValue = (float)Mathf.Round((vert.z - minMeshValue) * 10000) / 10000;
						vertOffset.z = offsetValue;
					}
					else if (extrusionAxis.Equals(ExtrusionAxis.X))
					{
						offsetValue = (float)Mathf.Round((vert.x - minMeshValue) * 10000) / 10000;
						vertOffset.x = offsetValue;
					}

					distinctVertexPositions.Add(offsetValue);
					verticesOffset.Add(vertOffset);
				}

				distinctVertexPositions = distinctVertexPositions.Distinct().ToList();
				distinctVertexPositions.Sort();

				//For mesh collider
				MeshLength = distinctVertexPositions[distinctVertexPositions.Count - 1];
				MeshHeight = Mathf.Abs(heightOfMeshMinValue) + Mathf.Abs(heightOfMeshMaxValue);
				MeshWidth = Mathf.Abs(widthOfMeshMinValue) + Mathf.Abs(widthOfMeshMaxValue);

				CustomWidthOfCollider = MeshWidth;
				CustomHeightOfCollider = MeshHeight;

			}
		}
		/// <summary>
		/// Extrudes the mesh along the path
		/// </summary>
		public void UpdateMesh()
		{
			if (customMesh == null)
			{
				Debug.LogError("No Mesh chosen");
				return;
			}

			if (customMaterial == null)
			{
				Debug.LogWarning("No material chosen");
			}

			GetPointsAndTangents();
			PlaceMeshesAlongPath();

			//Determine all world positions and related vectors needed to extrude the mesh along the path
			void GetPointsAndTangents()
			{
				points = new List<Vector3>();
				tangents = new List<Vector3>();
				binorms = new List<Vector3>();
				normals = new List<Vector3>();

				//Get references points along for each segment (one segment is one Bezier curve)
				List<float[]> evenlySpacedReferenceLengths = pathCreator.bezierPath.ReferenceLengths();

				//Get the length of each segment
				List<float> segmentLengths = new List<float>();
				foreach (var segment in evenlySpacedReferenceLengths)
				{
					segmentLengths.Add(segment[segment.Length - 1]);
				}

				//Get all distances along the path where vertices should be placed.
				List<float[]> distances = pathCreator.bezierPath.MeshLengthsOnCurve(distinctVertexPositions, segmentLengths);

				List<float[]> tvalues = new List<float[]>();

				//Calculate t-values for each unique vertex point along the path
				for (int i = 0; i < evenlySpacedReferenceLengths.Count; i++)
				{
					List<float> t = new List<float>();
					for (int j = 0; j < distances[i].Length; j++)
					{
						t.Add(pathCreator.bezierPath.CalculatetValue(evenlySpacedReferenceLengths[i], distances[i][j]));
					}
					tvalues.Add(t.ToArray());
				}

				//Get the world position and corresponding tangent on the path needed
				for (int i = 0; i < pathCreator.bezierPath.NumSegments; i++)
				{
					int numPoints = 0;
					Vector3[] p = pathCreator.bezierPath.GetPointsInSegment(i);
					for (int j = 0; j < tvalues[i].Length; j++)
					{

						points.Add(pathCreator.bezierPath.GetPoint(p, tvalues[i][j]));
						tangents.Add(pathCreator.bezierPath.GetTangent(p, tvalues[i][j]));
						numPoints++;
					}

				}

				//Average the tangents between neighboring points so the mesh lies flat on the path
				tangentsAveraged = new List<Vector3>();

				for (int i = 0; i < tangents.Count; i++)
				{
					if (i == 0 || i == tangents.Count - 1)
					{
						tangentsAveraged.Add(tangents[i]);
					}
					else tangentsAveraged.Add(tangents[i - 1] + tangents[i]);

					tangentsAveraged[i] = tangentsAveraged[i].normalized;
				}

				tangents = tangentsAveraged;

				//Binormals
				for (int i = 0; i < tangents.Count; i++)
				{
					binorms.Add(GetBinormal(tangents[i]));
				}

				//Normals
				for (int i = 0; i < binorms.Count; i++)
				{
					normals.Add(GetNormal(tangents[i], binorms[i]));
				}

				if (path.isClosedLoop)
				{
					points[points.Count - 1] = points[0];
					tangents[tangents.Count - 1] = tangents[0];
					binorms[binorms.Count - 1] = binorms[0];
					normals[normals.Count - 1] = normals[0];
				}

			}

			void PlaceMeshesAlongPath()
			{
				List<Vector3> verticesOfTotalMesh = new List<Vector3>();
				List<Vector3> normalsOfTotalMesh = new List<Vector3>();
				Quaternion[] orientation = new Quaternion[tangents.Count];
				Vector3[] verticesWithExtrusionAxisZero = new Vector3[verticesOffset.Count];

				//Populate an array that tells us at what distance each vertex should be placed
				int[] distanceIndexOfEachVertex = new int[verticesOffset.Count];

				for (int i = 0; i < verticesOffset.Count; i++)
				{
					float vertexValue = 0;

					if (extrusionAxis.Equals(ExtrusionAxis.Z))
					{
						vertexValue = verticesOffset[i].z;

					}
					else if (extrusionAxis.Equals(ExtrusionAxis.X))
					{
						vertexValue = verticesOffset[i].x;
					}

					for (int j = 0; j < distinctVertexPositions.Count; j++)
					{
						if (Mathf.Abs(vertexValue - distinctVertexPositions[j]) < 0.001f)
						{  // To avoid problems with rounding errors 
							distanceIndexOfEachVertex[i] = j;
						}

					}
				}

				//Get the orientation of each point on the path, i.e, the quaternion that is used to rotate the mesh vertex along the path
				for (int i = 0; i < tangents.Count; i++)
				{
					orientation[i] = GetOrientation(tangents[i], binorms[i]);
				}

				if (path.isClosedLoop)
				{
					orientation[orientation.Length - 1] = orientation[0];
				}

				//Populate an array that is same as mesh.vertices except is has 0 component in the extrusion axis
				//Makes it easier to determine the displacement of each vertex on the path
				for (int i = 0; i < verticesOffset.Count; i++)
				{
					verticesWithExtrusionAxisZero[i] = (verticesOffset[i]);

					if (extrusionAxis.Equals(ExtrusionAxis.Z))
					{
						verticesWithExtrusionAxisZero[i].z = 0;
					}
					else if (extrusionAxis.Equals(ExtrusionAxis.X))
					{
						verticesWithExtrusionAxisZero[i].x = 0;

					}

				}

				if (path.isClosedLoop)
				{
					verticesWithExtrusionAxisZero[verticesWithExtrusionAxisZero.Length - 1] = verticesWithExtrusionAxisZero[0];
				}

				int meshNumber = 0;

				//Determine the world position and corresponding normal of each point of the total mesh on the path
				for (int i = 0; i < pathCreator.bezierPath.NumSegments; i++)
				{
					//For every mesh on current segment
					for (int k = meshNumber; k < meshNumber + pathCreator.bezierPath.NumMeshesPerSegment[i]; k++)
					{
						for (int j = 0; j < verticesOffset.Count; j++)
						{
							//The distance in the extrusion axis of this vertex
							int d = distanceIndexOfEachVertex[j];

							//World position of current vertex on the path
							Vector3 newPos = orientation[d + k * (distinctVertexPositions.Count - 1)] * verticesWithExtrusionAxisZero[j] + points[d + k * (distinctVertexPositions.Count - 1)] - gameObject.transform.position;
							verticesOfTotalMesh.Add(newPos);

							//Normal of current vertex
							Vector3 newNorm = orientation[d + k * (distinctVertexPositions.Count - 1)] * customMesh.normals[j];
							normalsOfTotalMesh.Add(newNorm);

						}

					}

					meshNumber += pathCreator.bezierPath.NumMeshesPerSegment[i];
				}

				if (path.isClosedLoop)
				{
					verticesOfTotalMesh[verticesOfTotalMesh.Count - 1] = verticesOfTotalMesh[0];
					normalsOfTotalMesh[normalsOfTotalMesh.Count - 1] = normalsOfTotalMesh[0];
				}

				var numMeshes = pathCreator.bezierPath.NumMeshes;
				var triangleLength = customMesh.triangles.Length;
				var uvLength = customMesh.uv.Length;

				//Triangles
				List<int> tris = new List<int>();

				for (int i = 0; i < numMeshes; i++)
				{
					for (int j = 0; j < triangleLength; j++)
					{
						tris.Add(customMesh.triangles[j] + verticesOffset.Count * i);
					}
				}

				//The UVs
				List<Vector2> UVs = new List<Vector2>();

				for (int i = 0; i < numMeshes; i++)
				{
					for (int j = 0; j < uvLength; j++)
					{
						UVs.Add(customMesh.uv[j]);
					}
				}

				if (path.isClosedLoop)
				{
					tris[tris.Count - 1] = tris[0];
					UVs[UVs.Count - 1] = UVs[0];
				}

				//Make the total mesh
				meshCache.Clear();

				meshCache.SetVertices(verticesOfTotalMesh);
				meshCache.SetNormals(normalsOfTotalMesh);

				meshCache.uv = UVs.ToArray();
				meshCache.triangles = tris.ToArray();
				meshCache.RecalculateBounds();

				VertCount = verticesOfTotalMesh.Count;
			}

		}

		Quaternion GetOrientation(Vector3 tng, Vector3 binorm)
		{
			Vector3 normal = GetNormal(tng, binorm);  // This is causing the upside down stuff
			if (extrusionAxis.Equals(ExtrusionAxis.Z))
			{
				return Quaternion.LookRotation(tng, normal);

			}
			else if (extrusionAxis.Equals(ExtrusionAxis.X))
			{
				return Quaternion.LookRotation(-binorm, normal);
			}

			return Quaternion.identity;

		}

		Quaternion GetOrientation(Vector3 tng)
		{
			Vector3 binorm = Vector3.Cross(tng, Vector3.up);
			Vector3 nrm = GetNormal(tng, binorm);

			return Quaternion.LookRotation(tng, nrm);

		}

		Vector3 GetBinormal(Vector3 tangent)
		{
			return Vector3.Cross(Vector3.up, tangent).normalized;
		}

		Vector3 GetNormal(Vector3 tng, Vector3 binorm)
		{
			return Vector3.Cross(tng, binorm).normalized;
		}

		// Add MeshRenderer and MeshFilter components to this gameobject if not already attached
		void AssignMeshComponents()
		{

			if (meshHolder == null)
			{
				meshHolder = new GameObject("Custom Mesh Holder");
			}

			meshHolder.transform.rotation = pathCreator.transform.rotation;
			meshHolder.transform.position = pathCreator.transform.position;
			meshHolder.transform.localScale = Vector3.one;

			// Ensure mesh renderer and filter components are assigned
			if (!meshHolder.gameObject.GetComponent<MeshFilter>())
			{
				meshHolder.gameObject.AddComponent<MeshFilter>();
			}
			if (!meshHolder.GetComponent<MeshRenderer>())
			{
				meshHolder.gameObject.AddComponent<MeshRenderer>();
			}

			meshRenderer = meshHolder.GetComponent<MeshRenderer>();
			meshFilter = meshHolder.GetComponent<MeshFilter>();
			if (meshCache == null)
			{
				meshCache = new Mesh();
			}
			meshFilter.sharedMesh = meshCache;
		}

		void AssignMaterials()
		{
			if (customMaterial != null)
			{
				meshRenderer.sharedMaterials = new Material[] { customMaterial };
				meshRenderer.sharedMaterials[0].mainTextureScale = new Vector3(1, textureTiling);
			}
		}
	}
}
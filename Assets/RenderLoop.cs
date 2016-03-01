using UnityEngine;
using System.Collections;
using UnityEditor;

/**
 * Name: Aditya Nivarthi
 * Description: This file demonstrates the ray tracing functions and calculations of the rays to get light values.
 * Note: Please see the readme.md file for additional resources and information about this project.
 * Email: anivarthi@wpi.edu (anivarthi)
 */


/**
 * This class is used to represent a triangle with 3 points and 3 edges calculated from the points.
 */
public class Triangle
{
	// Edges of the triangle
	public Vector3 e12, e13, e23;

	// Vertices of the triangle
	public Vector3 p1, p2, p3;

	/**
	 * Creates a Triangle instance with the 3 points.
	 * 
	 * @param p1 The first point of the triangle.
	 * @param p2 The second point of the triangle.
	 * @param p3 The third point of the triangle.
	 */
	public Triangle (Vector3 p1, Vector3 p2, Vector3 p3)
	{
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
		computeEdges ();
	}

	/**
	 * Computes the edges of the triangle.
	 */
	void computeEdges ()
	{
		e12 = p2 - p1;
		e13 = p3 - p1;
		e23 = p3 - p2;
	}
}

/**
 * Main class for the ray tracer.
 */
public class RenderLoop : MonoBehaviour
{
	// ALL public variables here can be modified from the Unity editor
	// Those specified MUST be set from the Unity editor

	// Scene objects
	// These must be set from the Unity editor
	public Transform point_light_1; // First light in the scene
	public Transform point_light_2; // Second light in the scene
	public Camera scene_camera; // Unity camera in the scene
	public GameObject scene_objects; // Game object with the scene objects as child objects

	// Texture for cube
	// This must be set from the Unity editor
	public Texture2D cube_texture;
	
	// Class with the image plane
	RaytracedCamera raytrace_camera; 

	// Color definitions
	public Color ambient_color = new Color (0.0f, 0.0f, 0.0f);
	public Color diffuse_color = new Color (0.4f, 0.4f, 0.4f);
	public Color specular_color = new Color (0.7f, 0.7f, 0.7f);
	public Color background_color = Color.red;

	// Specular power constant
	public float specular_power = 27.8974f;

	// Light attenuation constants
	public float attenuation_constant = 10.0f;
	public float attenuation_exponent = 2.0f;

	// Debug factor for skipping columns and rows in rendering
	public int DEBUG_FACTOR = 1;

	// Column counter
	int column_index = 0;

	// Use this for initialization
	void Start ()
	{	
		// Load the cube texture from the Unity editor
		cube_texture = AssetDatabase.LoadAssetAtPath<Texture2D> ("Assets/WoodCrate_D.tif");

		// Get access to the image plane class on startup.
		raytrace_camera = GetComponent<RaytracedCamera> ();
	}

	/**
	 * Updates the screen. Draws the columns and rows of the image based on the ray tracer.
	 */
	void Update ()
	{
		// Cycle trhough the rows of the image plane only here in this loop.
		// Colums will be iterated as part of the update function.
		for (int j = 0; j < raytrace_camera.height; j+=DEBUG_FACTOR) {
			traceRayLocation (column_index, j);
		}

		// Account for debug factor
		column_index += DEBUG_FACTOR;

		if (column_index > raytrace_camera.width) {
			print ("Completed columns");

			// Reset the col counter as we have rendered them all
			column_index = 0;
		}
	}

	/**
	 * Calls sampleRayTriangle with a recurse level for reflectance.
	 * 
	 * @param ray The ray from the camera to the collider hit.
	 * @param go The game object being hit.
	 * @param color The output color that will be set by sampleRayTriangle()
	 * @param recurse_level The number of times to calculate the reflectance. This is used to render mirror and reflectance.
	 * @param texcoor The texture coordinates of the hit on the object.
	 *
	 * @return true if the game object was actually intersected by the ray, false otherwise.
	 * @output color The color to use for that pixel.
	 */
	bool traceRayLocationRay(Ray ray, GameObject go, out Color color, int recurse_level, Vector2 texcoord) {
		if (!sampleRayTriangle (ray, go, out color, recurse_level, texcoord)) {
			// Keep background color
			color = background_color;
			return false;
		} else {
			// Triangle was found, so we are done
			return true;
		}
	}

	/**
	 * This function sends rays out into the scene and determines if we have hit
	 * a bounding geometry (collider). The hit class holds a bunch of info regarding
	 * the object it hit including hit location, bounding geometry, link to the gameObject
	 * and actual mesh information. For more info read :
	 * http://docs.unity3d.com/ScriptReference/RaycastHit.html
	 * http://docs.unity3d.com/ScriptReference/Physics.Raycast.html
	 * 
	 * @param row The row of the image to render.
	 * @param col The column of the image to render.
	 */
	void traceRayLocation (int row, int col)
	{
		// Set the background color
		Color color = background_color;

		// Get the ray from the camera to the pixel
		Ray ray = scene_camera.ScreenPointToRay (new Vector3 (row, col, 0));

		// Get all the hits of that ray from the camera
		RaycastHit[] hits = Physics.RaycastAll (ray);

		// Set recursion level
		int recurse = 0;

		// Loop through all the hits of the ray. If it doesn't match a mesh triangle, skip it
		for (int i = 0; i < hits.Length; i++) {
			// Set reflecive mirroring for only the sphere
			if (hits[i].collider.gameObject.name.Equals("Sphere")) {
				recurse = 1;
			} else {
				recurse = 0;
			}

			// If the collision matches a triangle on the mesh of the collider, we found a hit
			if(traceRayLocationRay(ray, hits[i].collider.gameObject, out color, recurse, hits[i].textureCoord)) {
				break;
			}
		}

		// Update the image plane with the right color
		raytrace_camera.updateTexture (row, col, color);
	}

	/**
	 * This function should be used to iterate through all the triangles inside a
	 * bounding geometry, testing to find the best triangle intersection. If it is found
	 * the normal should be calculated and you should then determine how to calculate
	 * the reflectance at this point (determineReflectance()).
	 * 
	 * @param ray The ray from the camera to the collider hit.
	 * @param go The game object being hit.
	 * @param color The output color to set if a triangle is hit on the object.
	 * @param recurse_level The number of levels to recurse for calculating reflectance.
	 * @param texcoor The texture coordinates of the hit on the object.
	 *
	 * @return true if the game object was actually intersected by the ray, false otherwise.
	 * @output col The color for that pixel is returned in this variable.
	 */
	bool sampleRayTriangle (Ray ray, GameObject go, out Color color, int recurse_level, Vector2 texcoor)
	{
		// Set color by default to background color
		Color ret_color = background_color;

		// Distance to hit point
		float min_dist = Mathf.Infinity;
		Vector3 hit_point = Vector3.zero;
		Vector3 close_norm = Vector3.zero;

		// Get the mesh from the game object
		Mesh go_mesh = go.GetComponent<MeshFilter> ().mesh;
		int[] triangles = go_mesh.triangles;
		Vector3[] verts = go_mesh.vertices;
		Vector3[] norms = go_mesh.normals;

		// Create each of the triangles to test intersection
		for (int i = 0; i < triangles.Length; i += 3) {
			Triangle tri = new Triangle (verts [triangles [i]], verts [triangles [i + 1]], verts [triangles [i + 2]]);
			float[] weights = new float[3];

			// Get the transformed ray
			Ray trans_ray = new Ray (go.transform.InverseTransformPoint (ray.origin), go.transform.TransformDirection (ray.direction));

			// Get the distance from the ray origin to the hit point
			float current_dist = interSectionTest (trans_ray, tri, ref weights);

			// Get the minimum distance to the first triangle hit and calculate the normal and hit point for that distance
			if (current_dist < min_dist) {
				min_dist = current_dist;
				hit_point = go.transform.TransformPoint (trans_ray.origin + min_dist * trans_ray.direction);
				close_norm = weights [0] * norms [triangles [i]] + weights [1] * norms [triangles [i + 1]] + weights [2] * norms [triangles [i + 2]];
			}
		}

		// Get all the lights in the scene
		Light[] lights = FindObjectsOfType (typeof(Light)) as Light[];
		bool ret_bool = false;

		// Check if distance is an actual value and we have a triangle hit
		if (min_dist < Mathf.Infinity) {
			ret_color = ambient_color;
			foreach (Light light in lights) {
				// Get the vector from the light to the hit point
				Vector3 lv = light.transform.position - hit_point;

				// Check if the point is in a shadow
				if (!isInShadow (lv, hit_point, close_norm)) {
					ret_color += determineReflectance (ray, hit_point, close_norm);
					ret_bool = true;
				} else {
					ret_bool = true;
				}
			}
		}

		// If we are supposed to process the mirroring reflectance
		if (recurse_level > 0) {
			// Get the reflected ray over the normal of the hit point
			Color temp = background_color;
			Vector3 reflectance = Vector3.Normalize(ray.direction) - 2 * (Vector3.Dot (Vector3.Normalize(ray.direction), close_norm)) * close_norm;
			Ray reflect = new Ray(hit_point, reflectance);

			// Get all the hits of that ray from the camera
			RaycastHit[] hits = Physics.RaycastAll (reflect);

			// Check the collisions to match with triangles of the collider's mesh
			for (int i = 0; i < hits.Length; i++) {
				if(traceRayLocationRay(reflect, hits[i].collider.gameObject, out temp, --recurse_level, hits[i].textureCoord)) {
					break;
				}
			}

			// Add 50 percent of the mirroring effect to reduce shininess
			ret_color += 0.5f * temp;
		}

		// If we are supposed to map the box texture to the CUBE object
		if (go.name.Equals("Cube")) {
			ret_color += bilinearInterpolation(texcoor.x, texcoor.y);
		}

		// Set the color and return it
		color = ret_color;
		return ret_bool;
	}

	/**
	 * Calculates the bilinear interpolation of the given texture coordinates. Outputs the color of that
	 * coordinate on the CUBE texture, or the weighted average of the four nearest pixels.
	 * 
	 * @param u The x texture coordinate.
	 * @param v The y texture coordinate.
	 *
	 * @return The color of the texture at the specified texture coordinates.
	 */
	Color bilinearInterpolation(float tex_u, float tex_v) {
		// Store the width and height of the texture
		float tex_width = cube_texture.width;
		float tex_height = cube_texture.height;

		// Get the normalized values in relation to the texture
		float u = (tex_u * tex_width) - 0.5f;
		float v = (tex_v * tex_height) - 0.5f;

		// Get the normalized floored values in relation to the texture
		int x = Mathf.FloorToInt (u);
		int y = Mathf.FloorToInt (v);

		// Get the ratios of the coordinates in relation to the texture, and the inverses with 1 as the max
		float u_ratio = u - x;
		float v_ratio = v - y;
		float u_opposite = 1.0f - u_ratio;
		float v_opposite = 1.0f - v_ratio;

		Color result = (cube_texture.GetPixel(x, y) * u_opposite + cube_texture.GetPixel(x + 1, y) * u_ratio) * v_opposite + (cube_texture.GetPixel(x, y + 1) * u_opposite + cube_texture.GetPixel(x + 1, y + 1) * u_ratio) * v_ratio;

		return result;
	}

	/**
	 * This function determines if there is an intersection with a triangle and a ray (passed in as params).
	 * The return value is the distance of the intersection (i.e., t_hit). The parameters for weights should
	 * be filled in (notice it is "pass by reference") with the trilinear interpolation values (barycentric coordinate)
	 * to be used to determine the normal, which should be done outside this function.
	 * 
	 * @param ray The ray from the camera to the triangle.
	 * @param tri The triangle on the object to determine intersection with.
	 * @param weights[] The array of weights for the Moller Trumbore method of calculating intersection passed by reference to be filled in.
	 *
	 * @return The distance to the hit point if the ray intersects the triangle, or Mathf.Infinity if no intersection happened.
	 */
	float interSectionTest (Ray ray, Triangle tri, ref float[] weights)
	{
		float dist = Mathf.Infinity;
		float determinant = 0.0f;
		Vector3 p;
		Vector3 q;
		Vector3 t;

		p = Vector3.Cross (ray.direction, tri.e13);
		determinant = Vector3.Dot (tri.e12, p);

		// Get the vector from the origin of the ray to the first point of the triangle
		t = ray.origin - tri.p1;

		// Calculate U
		weights [1] = Vector3.Dot (t, p) / determinant;

		q = Vector3.Cross (t, tri.e12);

		// Calculate V
		weights [2] = Vector3.Dot (ray.direction, q) * 1 / determinant;

		// Calculate W from U and V
		weights [0] = 1 - weights [1] - weights [2];

		dist = Vector3.Dot (tri.e13, q) / determinant;

		// for reasons related to precision, it is good to make sure we do not have really small
		// values that could be false positives.
		double eps1 = 1e-7;
		double eps2 = 1e-10;
		if ((determinant <= eps1) || (weights [0] < -eps2) || (weights [1] <= -eps2) || (weights [2] <= -eps2)) {
			return Mathf.Infinity; //ray paralle to tri, or intersection is outside or behind the triangle.
		} else {
			return dist;
		}
	}

	/**
	 * This function is responsible for determining the reflectance at a given location
	 * using the normal and the hit point. It also takes into account the light attenuation
	 * based on the distance to each light.
	 * 
	 * @param ray The ray from the camera to the hit point.
	 * @param hit_point The hit point on the object to determine color for.
	 * @param normal The normal at the hit point.
	 *
	 * @return The color of the reflectance based on the lights.
	 */
	Color determineReflectance (Ray ray, Vector3 hit_point, Vector3 normal)
	{
		// Set default shadow color to ambient
		Color ret_color = ambient_color;

		// Get all lights in scene
		Light[] lights = FindObjectsOfType (typeof(Light)) as Light[];

		// Normalize hit point normal
		Vector3 normal_norm = Vector3.Normalize (normal);

		// Compute Blinn Phong
		foreach (Light l in lights) {
			// Light direction
			Vector3 light = l.transform.position - hit_point;
			float distance = light.magnitude;

			// Light attenuation to reduce light intensity based on distance
			float attenuation = determineAttenuation(distance);

			Vector3 light_norm = Vector3.Normalize(light);

			// Half way vector for Blinn Phong
			Vector3 halfDir = Vector3.Normalize (-ray.direction + light_norm);
			float diffuse = Mathf.Max (0, Vector3.Dot (light_norm, normal_norm));
			float specular = Mathf.Pow (Mathf.Max (0, Vector3.Dot (halfDir, normal_norm)), specular_power);

			// No ambient here because it is already added above
			ret_color += attenuation * (diffuse_color * diffuse + specular_color * specular);
		}

		return ret_color;
	}

	/**
	 * This function returns an attenuation based on the distance of the light.
	 * 
	 * @param light_dist The distance of the light from the hit point.
	 *
	 * @return The light attenuation value to multiply by the diffuse and specular components.
	 */
	float determineAttenuation (float light_dist) {
		float attenuation = (Mathf.Pow(light_dist, -attenuation_exponent));
		attenuation *= attenuation_constant;
		return attenuation;
	}

	/**
	 * Determines if the hit point is in a shadow or not. Being in shadow constitutes as calculating a ray to the
	 * light source and determining if anything is hit in between the two points.
	 * 
	 * @param light_vector The vector from the light to the hit point.
	 * @param hit_point The hit point of the ray on the object.
	 * @param normal The normal at the hit point.
	 *
	 * @return true if the point is in a shadow, false otherwise.
	 */
	bool isInShadow (Vector3 light_vector, Vector3 hit_point, Vector3 normal)
	{
		// If the dot product is basically 0, point is not in shadow
		if (Vector3.Dot (light_vector, normal) < 0) {
			return false;
		}
		
		// Epsilon value for preventing self colliding shadows
		Vector3 epsilon = 0.001f * scene_camera.transform.position;

		// Ray from hit point to light
		Ray ray = new Ray (hit_point - epsilon, light_vector);

		// Check if point is in shadow of the other objects
		for (int index = 0; index < scene_objects.transform.childCount; index++) {
			GameObject obj = scene_objects.transform.GetChild (index).gameObject;
			Mesh go_mesh = obj.GetComponent<MeshFilter> ().mesh;
			int[] triangles = go_mesh.triangles;
			Vector3[] verts = go_mesh.vertices;

			for (int i = 0; i < triangles.Length; i += 3) {
				Triangle tri = new Triangle (verts [triangles [i]], verts [triangles [i + 1]], verts [triangles [i + 2]]);
				float[] weights = new float[3];

				Ray trans_ray = new Ray (obj.transform.InverseTransformPoint (ray.origin), obj.transform.TransformDirection (ray.direction));

				// Call intersection test again
				float tmp_dist = interSectionTest (trans_ray, tri, ref weights);
				if (tmp_dist > 0.001 && tmp_dist < 1) {
					return true;
				}
			}
		}

		return false;
	}
}

using UnityEngine;
using System.Collections;
/**
 * Class that holds the image plane for the ray traced camera.
 * No modifications are needed for this class.
 * 
 * */
public class RaytracedCamera : MonoBehaviour {

	Texture2D viewPlaneTexture;
	public int width;
	public int height;
	public GUITexture viewPlane;

	void Start () {
		height = Screen.height;
		width = Screen.width;


		viewPlaneTexture = new Texture2D(width,height, TextureFormat.RGB24,false,true);

		// initialize our view plane texture to white
		for ( int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
				updateTexture(i,j,new Color(0,0,0));

		//make sure we have the viewplane
		if(viewPlane!=null)
			viewPlane.pixelInset = new Rect(0,0, width, height);
		else
			viewPlane = GetComponent<GUITexture>();

		viewPlane.texture = viewPlaneTexture;
	}
	public void updateTexture(int row, int col, Color c)
	{
		viewPlaneTexture.SetPixel(row, col, c);
	}
	// Update is called once per frame
	void Update () {
		viewPlaneTexture.Apply();
	}
}

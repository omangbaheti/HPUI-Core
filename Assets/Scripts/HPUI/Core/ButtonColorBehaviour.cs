﻿using UnityEngine;

namespace HPUI.Core
{
    public class ButtonColorBehaviour : MonoBehaviour
    {
	public Color highlightColor;
	// private Color secondaryHighlightColor;
	// public Color sucessHighlightColor;
	// public Color selectionColor;
	private Color defaultColor;
	//private Color secondaryDefaultColor;
	public SpriteRenderer spriteRenderer;
	public bool externalSpriteRender {get; private set;}
	// Start is called before the first frame update
	void Start()
	{
	    if (!spriteRenderer)
	    {
		spriteRenderer = GetComponent<SpriteRenderer>();
		externalSpriteRender = false;
	    }
	    else
	    {
		externalSpriteRender = true;
	    }
	    defaultColor = spriteRenderer.color;
	    //secondaryDefaultColor = spriteRenderer.color;
	    //secondaryHighlightColor = highlightColor;
	}

	// Update is called once per frame
	public void InvokeColorBehaviour()
	{
	    //Debug.Log("----------------------------------------------------------Color on " + GetComponentInParent<TransformLinker>().parent.name);
	    spriteRenderer.color = highlightColor;
	}

	public void ResetColor()
	{
	    //Debug.Log("----------------------------------------------------------Color off " + GetComponentInParent<TransformLinker>().parent.name);
	    spriteRenderer.color = defaultColor;
	}
    }
}

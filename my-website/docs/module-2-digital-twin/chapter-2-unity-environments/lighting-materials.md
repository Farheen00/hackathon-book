---
sidebar_position: 2
title: Lighting and Materials
description: Implementing realistic lighting and materials in Unity environments
id: ch2-s5-lighting-materials
---

# Lighting and Materials

This section covers implementing realistic lighting and materials in Unity environments. You'll learn how to create visually appealing scenes that provide accurate visual feedback for robot operations.

## Key Concepts

- Unity lighting system
- Material properties and shaders
- Light types and configurations
- Rendering pipeline optimization

## Diagram Descriptions

1. **Lighting Setup**: A visual representation of different light types (Directional, Point, Spot) and their effects on scene objects.
2. **Material Properties**: A diagram showing the key properties of Unity materials (albedo, metallic, smoothness, normal map).

## Content

Lighting and materials are crucial for creating realistic Unity environments. Proper lighting setup ensures that the environment looks realistic and provides accurate visual feedback for robot operations.

### Light Types

Unity supports several light types:

- Directional: Simulates sunlight or other distant light sources
- Point: Omnidirectional light that emits from a single point
- Spot: Conical light that illuminates a specific area
- Area: Rectangular or disc-shaped lights for realistic bounce lighting

### Material Properties

Materials define how surfaces interact with light:

- Albedo: Base color of the material
- Metallic: How metallic the surface appears
- Smoothness: How smooth or rough the surface is
- Normal map: Surface detail without additional geometry

## Example

Here's how to configure lighting and materials in Unity:

```csharp
// Example of configuring materials and lighting
public class LightingSetup : MonoBehaviour
{
    public Material robotMaterial;
    public Material environmentMaterial;
    public Light mainDirectionalLight;

    void Start()
    {
        ConfigureMaterials();
        SetupLighting();
    }

    void ConfigureMaterials()
    {
        // Set material properties
        robotMaterial.SetColor("_Color", Color.gray);
        robotMaterial.SetFloat("_Metallic", 0.5f);
        robotMaterial.SetFloat("_Smoothness", 0.5f);
    }

    void SetupLighting()
    {
        // Configure directional light
        mainDirectionalLight.color = Color.white;
        mainDirectionalLight.intensity = 1.0f;
        mainDirectionalLight.transform.rotation = Quaternion.Euler(50, -30, 0);
    }
}
```
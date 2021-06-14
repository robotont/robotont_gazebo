#version 150

// General functions

// Expand a range-compressed vector
vec3 expand(vec3 v)
{
	return (v - 0.5) * 2.0;
}

uniform vec4 lightDiffuse;
uniform vec4 lightSpecular;
uniform sampler2D diffuseMap;
uniform sampler2D normalMap;

in vec4 oUv0;
out vec3 oNormal;
in vec3 oLightDir;

in vec3 oTSLightDir;
//in vec3 oTSHalfAngle;

out vec4 fragColour;

// NOTE: GLSL does not have the saturate function.  But it is equivalent to clamp(val, 0.0, 1.0)

/* Fragment program which supports specular component */
void main()
{
	vec3 lightVec = normalize(oTSLightDir);
	vec4 diff = texture(diffuseMap, oUv0.xy);
	vec3 bumpVec = expand(texture(normalMap, oUv0.xy).xyz);
  fragColour = diff * 0.3 + 0.7 * diff * clamp(dot(bumpVec, lightVec), 0.0, 1.0);


//  fragColour = vec4(bumpVec,1.0) * clamp(dot(bumpVec, lightVec), 0.0, 1.0);
	// retrieve half angle and normalise
//	vec3 halfAngle = normalize(oTSHalfAngle);
	// get bump map vector, again expand from range-compressed

	// Pre-raise the specular exponent to the eight power
//	float specFactor = clamp(pow(dot(bumpVec, halfAngle),4.0), 0.0, 1.0);

	// Calculate dot product for diffuse
//	fragColour = (lightDiffuse * clamp(dot(bumpVec, lightVec), 0.0, 1.0)) + (lightSpecular * specFactor);
//fragColour = clamp(dot(bumpVec, lightVec), 0.0, 1.0);
//	fragColour = vec4(bumpVec,1.0);
}

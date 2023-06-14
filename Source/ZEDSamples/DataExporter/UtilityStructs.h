#pragma once

#include <string>
#include "CoreMinimal.h"
#include "Engine/World.h"


#include <ctime>
#include "cuda_runtime_api.h"

#ifdef _WIN32 //using windows
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include <winsock.h>
#include <intrin.h>
#include "TCHAR.h"
#include "pdh.h"
#pragma comment(lib, "Ws2_32.lib")
#include "Windows/HideWindowsPlatformTypes.h"
#endif

#include "Stereolabs/Public/Core/StereolabsBaseTypes.h"

#include "UtilityStructs.generated.h"


#define IS_MICROSECONDS false
#define GT_INVALID_VALUE -999.0
#define BBOX_MINIMUM_VOLUME 0.05f

template<typename T>
static FString EnumToString(const FString& enumName, const T value)
{
    UEnum* pEnum = FindObject<UEnum>(ANY_PACKAGE, *enumName);
    return *(pEnum ? pEnum->GetNameStringByIndex(static_cast<uint8>(value)) : "null");
}


enum class BODY_PARTS_POSE_34
{
    PELVIS = 0,
    NAVAL_SPINE = 1,
    CHEST_SPINE = 2,
    NECK = 3,
    LEFT_CLAVICLE = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    LEFT_HAND = 8,
    LEFT_HANDTIP = 9,
    LEFT_THUMB = 10,
    RIGHT_CLAVICLE = 11,
    RIGHT_SHOULDER = 12,
    RIGHT_ELBOW = 13,
    RIGHT_WRIST = 14,
    RIGHT_HAND = 15,
    RIGHT_HANDTIP = 16,
    RIGHT_THUMB = 17,
    LEFT_HIP = 18,
    LEFT_KNEE = 19,
    LEFT_ANKLE = 20,
    LEFT_FOOT = 21,
    RIGHT_HIP = 22,
    RIGHT_KNEE = 23,
    RIGHT_ANKLE = 24,
    RIGHT_FOOT = 25,
    HEAD = 26,
    NOSE = 27,
    LEFT_EYE = 28,
    LEFT_EAR = 29,
    RIGHT_EYE = 30,
    RIGHT_EAR = 31,
    LEFT_HEEL = 32,
    RIGHT_HEEL = 33,
    LAST = 34
};

enum class BODY_PARTS_POSE_18
{
    NOSE = 0,
    NECK = 1,
    RIGHT_SHOULDER = 2,
    RIGHT_ELBOW = 3,
    RIGHT_WRIST = 4,
    LEFT_SHOULDER = 5,
    LEFT_ELBOW = 6,
    LEFT_WRIST = 7,
    RIGHT_HIP = 8,
    RIGHT_KNEE = 9,
    RIGHT_ANKLE = 10,
    LEFT_HIP = 11,
    LEFT_KNEE = 12,
    LEFT_ANKLE = 13,
    RIGHT_EYE = 14,
    LEFT_EYE = 15,
    RIGHT_EAR = 16,
    LEFT_EAR = 17,
    LAST = 18
};


//Name of bones of the skeleton rig // for mixamo's models
const TArray<FName> targetBone_34 = {
    "Hips",
    "Spine1",
    "Spine2",
    "Neck",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "LeftHandMiddle1", 
    "LeftHandMiddle4",
    "LeftHandThumb4",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "RightHandMiddle1",
    "RightHandMiddle4",
    "RightHandThumb4",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToe_End",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToe_End",
    "Head",
    "not_found", //nose
    "LeftEye", // left eye
    "not_found", //left ear
    "RightEye", // right eye
    "not_found", //right ear
    "not_found", //left heel
    "not_found" //right heel
};

//Name of bones of the skeleton rig // for mixamo's models
const TArray<FName> targetBone_18 = {
    "not_found", //nose
    "Neck",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "RightEye", // right eye
    "LeftEye", // left eye
    "not_found", //right ear
    "not_found" //left ear
};


const TArray<float> occlusion_thresholds = {
    20, //Hips
    30, //Spine 1
    30, //Spine 2
    15, //Neck
    25, //LeftShoulder
    20, //LeftArm
    12, //LeftForeArm
    8, //LeftHand
    8, //LeftHandMiddle1
    5, //LeftHandMiddle4
    4, //LeftHandThumb4
    25, //RightShoulder
    20, //RightArm
    12, //RightForeArm
    8, //RightHand
    8, //RightHandMiddle1
    5, //RightHandMiddle4
    4, //RightHandThumb4
    25, //LeftUpLeg
    18, //LeftLeg
    12, //LeftFoot
    7, // LeftToe_End
    25, //RightUpLeg
    18, //RightLeg
    12, //RightFoot
    7, // RightToe_End
    15, //Head
    999, //
    7, //LeftEye
    999,
    7, //RightEye
    999,
    999,
    999
};

// enum to string fnct
static FString enumToString(BODY_PARTS_POSE_34 joint)
{
    FString tostring;

    switch (joint) {

    case BODY_PARTS_POSE_34::PELVIS:
        tostring = "PELVIS";
        break;
    case BODY_PARTS_POSE_34::NAVAL_SPINE:
        tostring = "NAVAL_SPINE";
        break;
    case BODY_PARTS_POSE_34::CHEST_SPINE:
        tostring = "CHEST_SPINE";
        break;
    case BODY_PARTS_POSE_34::NECK:
        tostring = "NECK";
        break;
    case BODY_PARTS_POSE_34::LEFT_CLAVICLE:
        tostring = "LEFT_CLAVICLE";
        break;
    case BODY_PARTS_POSE_34::LEFT_SHOULDER:
        tostring = "LEFT_SHOULDER";
        break;
    case BODY_PARTS_POSE_34::LEFT_ELBOW:
        tostring = "LEFT_ELBOW";
        break;
    case BODY_PARTS_POSE_34::LEFT_WRIST:
        tostring = "LEFT_WRIST";
        break;
    case BODY_PARTS_POSE_34::LEFT_HAND:
        tostring = "LEFT_HAND";
        break;
    case BODY_PARTS_POSE_34::LEFT_HANDTIP:
        tostring = "LEFT_HANDTIP";
        break;
    case BODY_PARTS_POSE_34::LEFT_THUMB:
        tostring = "LEFT_THUMB";
        break;
    case BODY_PARTS_POSE_34::RIGHT_CLAVICLE:
        tostring = "RIGHT_CLAVICLE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_SHOULDER:
        tostring = "RIGHT_SHOULDER";
        break;
    case BODY_PARTS_POSE_34::RIGHT_ELBOW:
        tostring = "RIGHT_ELBOW";
        break;
    case BODY_PARTS_POSE_34::RIGHT_WRIST:
        tostring = "RIGHT_WRIST";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HAND:
        tostring = "RIGHT_HAND";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HANDTIP:
        tostring = "RIGHT_HANDTIP";
        break;
    case BODY_PARTS_POSE_34::RIGHT_THUMB:
        tostring = "RIGHT_THUMB";
        break;
    case BODY_PARTS_POSE_34::LEFT_HIP:
        tostring = "LEFT_HIP";
        break;
    case BODY_PARTS_POSE_34::LEFT_KNEE:
        tostring = "LEFT_KNEE";
        break;
    case BODY_PARTS_POSE_34::LEFT_ANKLE:
        tostring = "LEFT_ANKLE";
        break;
    case BODY_PARTS_POSE_34::LEFT_FOOT:
        tostring = "LEFT_FOOT";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HIP:
        tostring = "RIGHT_HIP";
        break;
    case BODY_PARTS_POSE_34::RIGHT_KNEE:
        tostring = "RIGHT_KNEE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_ANKLE:
        tostring = "RIGHT_ANKLE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_FOOT:
        tostring = "RIGHT_FOOT";
        break;
    case BODY_PARTS_POSE_34::HEAD:
        tostring = "HEAD";
        break;
    case BODY_PARTS_POSE_34::NOSE:
        tostring = "NOSE";
        break;
    case BODY_PARTS_POSE_34::LEFT_EYE:
        tostring = "LEFT_EYE";
        break;
    case BODY_PARTS_POSE_34::LEFT_EAR:
        tostring = "LEFT_EAR";
        break;
    case BODY_PARTS_POSE_34::RIGHT_EYE:
        tostring = "RIGHT_EYE";
        break;
    case BODY_PARTS_POSE_34::RIGHT_EAR:
        tostring = "RIGHT_EAR";
        break;
    case BODY_PARTS_POSE_34::LEFT_HEEL:
        tostring = "LEFT_HEEL";
        break;
    case BODY_PARTS_POSE_34::RIGHT_HEEL:
        tostring = "RIGHT_HEEL";
        break;
    default:
        tostring = "WRONG JOINT NAME";
    }

    return tostring;
}

static FString enumToString(BODY_PARTS_POSE_18 joint)
{
    FString tostring;

    switch (joint) {

    case BODY_PARTS_POSE_18::LEFT_SHOULDER:
        tostring = "LEFT_SHOULDER";
        break;
    case BODY_PARTS_POSE_18::LEFT_ELBOW:
        tostring = "LEFT_ELBOW";
        break;

    case BODY_PARTS_POSE_18::LEFT_WRIST:
        tostring = "LEFT_WRIST";
        break;
    case BODY_PARTS_POSE_18::RIGHT_SHOULDER:
        tostring = "RIGHT_SHOULDER";
        break;
    case BODY_PARTS_POSE_18::RIGHT_ELBOW:
        tostring = "RIGHT_ELBOW";
        break;
    case BODY_PARTS_POSE_18::RIGHT_WRIST:
        tostring = "RIGHT_WRIST";
        break;
    case BODY_PARTS_POSE_18::LEFT_HIP:
        tostring = "LEFT_HIP";
        break;
    case BODY_PARTS_POSE_18::LEFT_KNEE:
        tostring = "LEFT_KNEE";
        break;
    case BODY_PARTS_POSE_18::LEFT_ANKLE:
        tostring = "LEFT_ANKLE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_HIP:
        tostring = "RIGHT_HIP";
        break;
    case BODY_PARTS_POSE_18::RIGHT_KNEE:
        tostring = "RIGHT_KNEE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_ANKLE:
        tostring = "RIGHT_ANKLE";
        break;
    case BODY_PARTS_POSE_18::NOSE:
        tostring = "NOSE";
        break;
    case BODY_PARTS_POSE_18::NECK:
        tostring = "NECK";
        break;
    case BODY_PARTS_POSE_18::LEFT_EYE:
        tostring = "LEFT_EYE";
        break;
    case BODY_PARTS_POSE_18::LEFT_EAR:
        tostring = "LEFT_EAR";
        break;
    case BODY_PARTS_POSE_18::RIGHT_EYE:
        tostring = "RIGHT_EYE";
        break;
    case BODY_PARTS_POSE_18::RIGHT_EAR:
        tostring = "RIGHT_EAR";
        break;
    default:
        tostring = "WRONG JOINT NAME";
    }
    return tostring;
}




static bool isInvalidValue(FVector in) {
    bool isInvalid = false;

    if (in.X == GT_INVALID_VALUE /* && in.Y <= INVALID_VALUE && in.Z <= INVALID_VALUE*/) {
        isInvalid = true;
    }

    return isInvalid;
}

static bool isInvalidValue(FQuat in)
{
    bool isInvalid = false;

    if (in.X <= GT_INVALID_VALUE && in.Y <= GT_INVALID_VALUE && in.Z <= GT_INVALID_VALUE && in.W <= GT_INVALID_VALUE) {
        isInvalid = true;
    }

    return isInvalid;
}

static FVector warpPoint_(FVector pt_curr, const FMatrix path, float scale = 1)
{
    FVector proj3D;
    const auto p_path = path.M;

    proj3D.X = ((pt_curr.X * p_path[0][0]) + (pt_curr.Y * p_path[0][1]) + (pt_curr.Z * p_path[0][2]) + p_path[0][3]) * scale;
    proj3D.Y = ((pt_curr.X * p_path[1][0]) + (pt_curr.Y * p_path[1][1]) + (pt_curr.Z * p_path[1][2]) + p_path[1][3]) * scale;
    proj3D.Z = ((pt_curr.X * p_path[2][0]) + (pt_curr.Y * p_path[2][1]) + (pt_curr.Z * p_path[2][2]) + p_path[2][3]) * scale;
    return proj3D;
}

static FVector2D projectPoint_(FVector pt, float fx, float fy, float cx, float cy) {
    FVector2D pxl = FVector2D::ZeroVector;

    pxl.X = (pt.X / pt.Z) * fx + cx;
    pxl.Y = (pt.Y / pt.Z) * fy + cy;

    return pxl;
}

static inline FVector2D checkVisible(float width, float height, FVector2D point)
{
    if (point.X < width && point.Y < height && point.X > 0 && point.Y > 0) {
        return point;
    }
    else {
        return FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
    }
}

static FVector convertFromImageToUnityCoordinateSystem(FVector in)
{
    FVector out(in.X, - in.Y, in.Z);
    out /= 100;

    return out;
}

static FVector convertFromImageToZEDIMUCoordinateSystem(FVector in)
{
    FVector out(in.Y, in.X, -in.Z);
    return out;
}

static FVector convertFromUUToUnityCoordinateSystem(FVector in, bool convertUnit = true)
{
    int factor = convertUnit ? 100 : 1;
    FVector out(in.Y, in.Z, in.X);
    out /= factor;

    return out;
}

static FTransform getCoordinateTransformConversion4f()
{
    FMatrix tmp, coordTransf;
    tmp.SetIdentity();
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[0][0] = 0;
    tmp.M[0][1] = 1;
    tmp.M[1][0] = 0;
    tmp.M[1][1] = 0;
    tmp.M[1][2] = -1;
    tmp.M[2][0] = 1;
    tmp.M[2][1] = 0;
    tmp.M[2][2] = 0;

    //getInverseCoordinateTransform
    coordTransf.M[1][1] = -1;

    return FTransform(coordTransf.Inverse()) * FTransform(tmp);
}

static FTransform convertFromUUToUnityCoordinateSystem(FTransform in)
{
    FTransform coordTransf = getCoordinateTransformConversion4f();
    FTransform out = (coordTransf * in * coordTransf.Inverse());

    return out;
}

static FVector convertFromUUToImageCoordinateSystem(FVector in)
{
    FVector out = FVector(in.Y, -in.Z, in.X);
    return out;
}

static FTransform convertFromUUToImageCoordinateSystem(FTransform in)
{
    FMatrix tmp;
    tmp.SetIdentity();
    FMatrix coordTransf;
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[0][0] = 0;    tmp.M[0][1] = 1;    tmp.M[0][2] = 0; //src is left handed - z - up
    tmp.M[1][0] = 0;    tmp.M[1][1] = 0;    tmp.M[1][2] = -1;
    tmp.M[2][0] = 1;    tmp.M[2][1] = 0;    tmp.M[2][2] = 0;

    coordTransf.SetIdentity(); // dest is IMAGE

    FTransform tf = FTransform(coordTransf.Inverse()) * FTransform(tmp);
    FTransform out = tf * in * tf.Inverse(); 

    return out;
}

static FTransform convertFromUnityToImageCoordinateSystem(FTransform in)
{
    FMatrix tmp;
    tmp.SetIdentity();
    FMatrix coordTransf;
    coordTransf.SetIdentity();

    //getCoordinateTransform
    tmp.M[1][1] = -1;

    coordTransf.SetIdentity(); // dest is IMAGE

    FTransform tf = FTransform(coordTransf.Inverse()) * FTransform(tmp);
    FTransform out = tf * in * tf.Inverse();

    return out;
}

static FVector camToWorld(FTransform camPose, FVector in) {
    //in = camPose.TransformPosition(in);
    in = camPose.GetRotation().RotateVector(in);
    in += camPose.GetTranslation();
    return in;
}

static FQuat camToWorld(FTransform camPose, FQuat in)
{
    in = camPose.TransformRotation(in);
    return in;
}

static FVector worldToCam(FTransform camPose, FVector in)
{
    camPose = camPose.Inverse();
    in = camPose.GetRotation().RotateVector(in);
    in += camPose.GetTranslation();
    return in;
}

static FQuat worldToCam(FTransform camPose, FQuat in)
{
    camPose = camPose.Inverse();
    in = camPose.TransformRotation(in);
    return in;
}


USTRUCT()
struct FJsonMatrix4x4
{
    GENERATED_BODY()

public:
    UPROPERTY()
    float e00 = 0.0f;

    UPROPERTY()
    float e01 = 0.0f;

    UPROPERTY()
    float e02 = 0.0f;

    UPROPERTY()
    float e03 = 0.0f;

    UPROPERTY()
    float e10 = 0.0f;

    UPROPERTY()
    float e11 = 0.0f;

    UPROPERTY()
    float e12 = 0.0f;

    UPROPERTY()
    float e13 = 0.0f;

    UPROPERTY()
    float e20 = 0.0f;

    UPROPERTY()
    float e21 = 0.0f;

    UPROPERTY()
    float e22 = 0.0f;

    UPROPERTY()
    float e23 = 0.0f;

    UPROPERTY()
    float e30 = 0.0f;

    UPROPERTY()
    float e31 = 0.0f;

    UPROPERTY()
    float e32 = 0.0f;
 
    UPROPERTY()
    float e33 = 0.0f;

    FJsonMatrix4x4() {
        e00 = GT_INVALID_VALUE;
        e01 = GT_INVALID_VALUE;
        e02 = GT_INVALID_VALUE; 
        e03 = GT_INVALID_VALUE;

        e10 = GT_INVALID_VALUE;
        e11 = GT_INVALID_VALUE;
        e12 = GT_INVALID_VALUE;
        e13 = GT_INVALID_VALUE;

        e20 = GT_INVALID_VALUE;
        e21 = GT_INVALID_VALUE;
        e22 = GT_INVALID_VALUE;
        e23 = GT_INVALID_VALUE;

        e30 = GT_INVALID_VALUE;
        e31 = GT_INVALID_VALUE;
        e32 = GT_INVALID_VALUE;
        e33 = GT_INVALID_VALUE;
    }

    FJsonMatrix4x4(FTransform transform)
    {
        //Convert to Unity Coordinate system (left handed Z up to left handed Y up)
        FTransform coordTransf = getCoordinateTransformConversion4f();
        FMatrix mat = (coordTransf * transform * coordTransf.Inverse()).ToMatrixWithScale();
        mat = mat.GetTransposed();

        e03 = mat.M[0][3] / 100.0;
        e13 = mat.M[1][3] / 100.0;
        e23 = mat.M[2][3] / 100.0;
        e33 = 1.0;

        e00 = mat.M[0][0];
        e01 = mat.M[0][1];
        e02 = mat.M[0][2];
        e10 = mat.M[1][0];
        e11 = mat.M[1][1];
        e12 = mat.M[1][2];
        e20 = mat.M[2][0];
        e21 = mat.M[2][1];
        e22 = mat.M[2][2];

        e30 = 0;
        e31 = 0;
        e32 = 0;
    }

    FTransform toTransform() {
        FTransform transform;

        FMatrix mat = FMatrix();
        mat.SetIdentity();

        mat.M[0][0] = e00;
        mat.M[0][1] = e01;
        mat.M[0][2] = e02;
        mat.M[0][3] = e03;
        mat.M[1][0] = e10;
        mat.M[1][1] = e11;
        mat.M[1][2] = e12;
        mat.M[1][3] = e13;
        mat.M[2][0] = e20;
        mat.M[2][1] = e21;
        mat.M[2][2] = e22;
        mat.M[2][3] = e23;
        mat.M[3][0] = e20;
        mat.M[3][1] = e31;
        mat.M[3][2] = e32;
        mat.M[3][3] = e33;


        transform.SetRotation(FQuat(mat));
        transform.SetTranslation(FVector(e03, e13, e23));
        //transform.SetFromMatrix(mat.GetTransposed());
        return transform;
    }
};


USTRUCT()
struct FJsonMetaData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString SequenceID = "0";

    UPROPERTY()
    int32 ZEDSerialNumber = 0;

    UPROPERTY()
    bool IsRealZED = false;

    UPROPERTY()
    bool IsRectified = true;

    UPROPERTY()
    int32 ImageWidth = 0;

    UPROPERTY()
    int32 ImageHeight = 0;

    UPROPERTY()
    int32 TargetFPS = 0;

    UPROPERTY()
    float InvalidValue = 0.0f;

    UPROPERTY()
    float Bbox3dMinimumVolume = 0.0f;

    UPROPERTY()
    bool IsMicroseconds = true;
};

USTRUCT()
struct FJsonFramePoseData
{
    GENERATED_BODY()

public:

    UPROPERTY()
   FJsonMatrix4x4 WorldPose;
};


USTRUCT()
struct FJsonBoundingBox3DData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FVector A;

    UPROPERTY()
    FVector B;

    UPROPERTY()
    FVector C;

    UPROPERTY()
    FVector D;

    UPROPERTY()
    FVector E;

    UPROPERTY()
    FVector F;

    UPROPERTY()
    FVector G;

    UPROPERTY()
    FVector H;

    FJsonBoundingBox3DData() {
        A = FVector(GT_INVALID_VALUE,GT_INVALID_VALUE,GT_INVALID_VALUE);
        B = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        C = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        D = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        E = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        F = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        G = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
        H = FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE);
    }
};

USTRUCT()
struct FJsonBoundingBox2DData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FVector2D A;

    UPROPERTY()
    FVector2D B;

    UPROPERTY()
    FVector2D C;

    UPROPERTY()
    FVector2D D;

    FJsonBoundingBox2DData()
    {
        A = FVector2D(GT_INVALID_VALUE,GT_INVALID_VALUE);
        B = FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
        C = FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
        D = FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
    }

};

USTRUCT()
struct FJsonSingleDetection
{
    GENERATED_BODY()
public:

    UPROPERTY()
    int ObjectID = 0;

    UPROPERTY()
    int ObjectType = 0;

    //UPROPERTY()
    //FVector Position3D_World_Floor = FVector::Zero();

    //UPROPERTY()
    //FVector Velocity3D_MPS = FVector::Zero();

    //UPROPERTY()
    //FVector Dimensions3D = FVector::Zero();

    //UPROPERTY()
    //FJsonBoundingBox3DData BoundingBox3D_World;

    //UPROPERTY()
    //FJsonBoundingBox3DData BoundingBox3D_World_Raw;

    //UPROPERTY()
    //FJsonBoundingBox2DData BoundingBox2D;

    //UPROPERTY()
    //FJsonBoundingBox2DData BoundingBox2D_Raw;

    //UPROPERTY()
    //FQuat GlobalRootOrientation = FQuat::Identity;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D;

    //UPROPERTY()
    //TArray<FVector2D> KeypointsConfidence2D;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D_34;

   // UPROPERTY()
    //TArray<FVector2D> KeypointsConfidence2D_34;

    UPROPERTY()
    TArray<FVector> Keypoints3D;

    //UPROPERTY()
    //TArray<FVector> KeypointsConfidence3D;

    UPROPERTY()
    TArray<FVector> Keypoints3D_34;

    //UPROPERTY()
    //TArray<FVector> KeypointsConfidence3D_34;

    //UPROPERTY()
    //TArray<FVector> LocalPositionPerJoint;

    //UPROPERTY()
    //TArray<FQuat> LocalOrientationPerJoint;
};

USTRUCT()
struct FJsonSkeletonData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FQuat GlobalRootOrientation = FQuat::Identity;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D;

    UPROPERTY()
    TArray<FVector2D> Keypoints2D_34;

    UPROPERTY()
    TArray<FVector> Keypoints3D;

    UPROPERTY()
    TArray<FVector> Keypoints3D_34;

    UPROPERTY()
    TArray<FVector> LocalPositionPerJoint;

    UPROPERTY()
    TArray<FQuat> LocalOrientationPerJoint;
};

USTRUCT()
struct FJsonFrameDetections
{
    GENERATED_BODY()

public :
    UPROPERTY()
        TArray<FJsonSingleDetection> ObjectDetections;
};

USTRUCT()
struct FJsonFrameData
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString ImageFileName;

    UPROPERTY()
    int32 FrameIndex = 0;

    UPROPERTY()
    uint64 EpochTimeStamp = 0;

    UPROPERTY()
    FJsonFrameDetections Detections;

    UPROPERTY()
    FJsonFramePoseData TrackedPose;
    
};

USTRUCT()
struct FJsonDataSet
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FJsonMetaData Metadata;

    UPROPERTY()
    FJsonMatrix4x4 InitialWorldPosition;

    UPROPERTY()
    TArray<FJsonFrameData> Frames;

    FJsonDataSet(){}
};

static FString SerializeAccuracyJson(FJsonDataSet data)
{
    FString OutputString;

    // create a Json object and add a string field
    TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

    ////////////////////////////////////
    /// METADATA ///////////////////////
    ////////////////////////////////////    

    TSharedPtr<FJsonObject> metadata = MakeShareable(new FJsonObject);
    metadata->SetStringField("SequenceID", data.Metadata.SequenceID);
    metadata->SetNumberField("ZEDSerialNumber", data.Metadata.ZEDSerialNumber);
    metadata->SetBoolField("IsRealZED", data.Metadata.IsRealZED);
    metadata->SetBoolField("IsRectified", data.Metadata.IsRectified);
    metadata->SetNumberField("ImageWidth", data.Metadata.ImageWidth);
    metadata->SetNumberField("ImageHeight", data.Metadata.ImageHeight);
    metadata->SetNumberField("TargetFPS", data.Metadata.TargetFPS);
    metadata->SetNumberField("InvalidValue", data.Metadata.InvalidValue);
    metadata->SetNumberField("Bbox3dMinimumVolume", data.Metadata.Bbox3dMinimumVolume);
    metadata->SetBoolField("IsMicroseconds", data.Metadata.IsMicroseconds);

    JsonObject->SetObjectField("Metadata", metadata);

    //// Initial World Position
    //TSharedPtr<FJsonObject> InitialWorldPosition = MakeShareable(new FJsonObject);
    //InitialWorldPosition->SetNumberField("e00", data.InitialWorldPosition.e00);
    //InitialWorldPosition->SetNumberField("e01", data.InitialWorldPosition.e01);
    //InitialWorldPosition->SetNumberField("e02", data.InitialWorldPosition.e02);
    //InitialWorldPosition->SetNumberField("e03", data.InitialWorldPosition.e03);
    //InitialWorldPosition->SetNumberField("e10", data.InitialWorldPosition.e10);
    //InitialWorldPosition->SetNumberField("e11", data.InitialWorldPosition.e11);
    //InitialWorldPosition->SetNumberField("e12", data.InitialWorldPosition.e12);
    //InitialWorldPosition->SetNumberField("e13", data.InitialWorldPosition.e13);
    //InitialWorldPosition->SetNumberField("e20", data.InitialWorldPosition.e20);
    //InitialWorldPosition->SetNumberField("e21", data.InitialWorldPosition.e21);
    //InitialWorldPosition->SetNumberField("e22", data.InitialWorldPosition.e22);
    //InitialWorldPosition->SetNumberField("e23", data.InitialWorldPosition.e23);
    //InitialWorldPosition->SetNumberField("e30", data.InitialWorldPosition.e30);
    //InitialWorldPosition->SetNumberField("e31", data.InitialWorldPosition.e31);
    //InitialWorldPosition->SetNumberField("e32", data.InitialWorldPosition.e32);
    //InitialWorldPosition->SetNumberField("e33", data.InitialWorldPosition.e33);

    //JsonObject->SetObjectField("InitialWorldPosition", InitialWorldPosition);

    TArray<TSharedPtr<FJsonValue>> Frames;
    for (auto frame : data.Frames) {

        TSharedPtr<FJsonObject> frameObj = MakeShareable(new FJsonObject);
        frameObj->SetStringField("ImageFileName", frame.ImageFileName);
        frameObj->SetNumberField("FrameIndex", frame.FrameIndex);
        frameObj->SetNumberField("EpochTimeStamp", frame.EpochTimeStamp);

        //////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////// ODOMETRY DATA ////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////

        TSharedPtr<FJsonObject> TrackedPose = MakeShareable(new FJsonObject);
        TSharedPtr<FJsonObject> WorldPose = MakeShareable(new FJsonObject);

        WorldPose->SetNumberField("e00", frame.TrackedPose.WorldPose.e00);
        WorldPose->SetNumberField("e01", frame.TrackedPose.WorldPose.e01);
        WorldPose->SetNumberField("e02", frame.TrackedPose.WorldPose.e02);
        WorldPose->SetNumberField("e03", frame.TrackedPose.WorldPose.e03);
        WorldPose->SetNumberField("e10", frame.TrackedPose.WorldPose.e10);
        WorldPose->SetNumberField("e11", frame.TrackedPose.WorldPose.e11);
        WorldPose->SetNumberField("e12", frame.TrackedPose.WorldPose.e12);
        WorldPose->SetNumberField("e13", frame.TrackedPose.WorldPose.e13);
        WorldPose->SetNumberField("e20", frame.TrackedPose.WorldPose.e20);
        WorldPose->SetNumberField("e21", frame.TrackedPose.WorldPose.e21);
        WorldPose->SetNumberField("e22", frame.TrackedPose.WorldPose.e22);
        WorldPose->SetNumberField("e23", frame.TrackedPose.WorldPose.e23);
        WorldPose->SetNumberField("e30", frame.TrackedPose.WorldPose.e30);
        WorldPose->SetNumberField("e31", frame.TrackedPose.WorldPose.e31);
        WorldPose->SetNumberField("e32", frame.TrackedPose.WorldPose.e32);
        WorldPose->SetNumberField("e33", frame.TrackedPose.WorldPose.e33);

        TrackedPose->SetObjectField("WorldPose", WorldPose);
        frameObj->SetObjectField("TrackedPose", TrackedPose);

        ////////////////////////////////////
        /// DETECTION DATA /////////////////
        ////////////////////////////////////

        TSharedPtr<FJsonObject> Detections = MakeShareable(new FJsonObject);

        TArray<TSharedPtr<FJsonValue>> ObjectDetections;

        for (auto detection : frame.Detections.ObjectDetections)
        {
            TSharedPtr<FJsonObject> singleDetectionObj = MakeShareable(new FJsonObject);

            singleDetectionObj->SetNumberField("ObjectID", detection.ObjectID);
            singleDetectionObj->SetNumberField("ObjectType", detection.ObjectType);

#if 0
            TArray<TSharedPtr<FJsonValue>> position3D;
            position3D.SetNum(3);
            position3D[0] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[0])));
            position3D[1] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[1])));
            position3D[2] = (MakeShareable(new FJsonValueNumber(detection.Position3D_World_Floor[2])));
            singleDetectionObj->SetArrayField("Position3D_World_Floor", position3D);

            TArray<TSharedPtr<FJsonValue>> velocity3D;
            velocity3D.SetNum(3);
            velocity3D[0] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[0])));
            velocity3D[1] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[1])));
            velocity3D[2] = (MakeShareable(new FJsonValueNumber(detection.Velocity3D_MPS[2])));
            singleDetectionObj->SetArrayField("Velocity3D_MPS", velocity3D);

            TArray<TSharedPtr<FJsonValue>> dimensions3D;
            dimensions3D.SetNum(3);
            dimensions3D[0] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[0])));
            dimensions3D[1] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[1])));
            dimensions3D[2] = (MakeShareable(new FJsonValueNumber(detection.Dimensions3D[2])));
            singleDetectionObj->SetArrayField("Dimensions3D", dimensions3D);

            ////////////////////////////////////
            /// 2D DATA ////// /////////////////
            ////////////////////////////////////

            TSharedPtr<FJsonObject> Bbox_2D = MakeShareable(new FJsonObject);
            TArray<TSharedPtr<FJsonValue>> A;
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.A[1])));
            Bbox_2D->SetArrayField("A", A);

            TArray<TSharedPtr<FJsonValue>> B;
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.B[1])));
            Bbox_2D->SetArrayField("B", B);

            TArray<TSharedPtr<FJsonValue>> C;
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.C[1])));
            Bbox_2D->SetArrayField("C", C);

            TArray<TSharedPtr<FJsonValue>> D;
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D.D[1])));
            Bbox_2D->SetArrayField("D", D);

            singleDetectionObj->SetObjectField("BoundingBox2D", Bbox_2D);

            TSharedPtr<FJsonObject> Bbox_2D_Raw = MakeShareable(new FJsonObject);
            A.Reset();
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.A[1])));
            Bbox_2D_Raw->SetArrayField("A", A);

            B.Reset();
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.B[1])));
            Bbox_2D_Raw->SetArrayField("B", B);

            C.Reset();
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.C[1])));
            Bbox_2D_Raw->SetArrayField("C", C);

            D.Reset();
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox2D_Raw.D[1])));
            Bbox_2D_Raw->SetArrayField("D", D);

            singleDetectionObj->SetObjectField("BoundingBox2D_Raw", Bbox_2D_Raw);


            ////////////////////////////////////
            /// 3D DATA ////////////////////////
            ////////////////////////////////////

            ////////////////////////////////////
            /// BOUNDING BOX  //////////////////
            ////////////////////////////////////

            TSharedPtr<FJsonObject> Bbox_3D = MakeShareable(new FJsonObject);
            A.Reset(3);
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[1])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.A[2])));
            Bbox_3D->SetArrayField("A", A);

            B.Reset(3);
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[1])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.B[2])));
            Bbox_3D->SetArrayField("B", B);

            C.Reset(3);
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[1])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.C[2])));
            Bbox_3D->SetArrayField("C", C);

            D.Reset(3);
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[1])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.D[2])));
            Bbox_3D->SetArrayField("D", D);

            TArray<TSharedPtr<FJsonValue>> E;
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[0])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[1])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.E[2])));
            Bbox_3D->SetArrayField("E", E);

            TArray<TSharedPtr<FJsonValue>> F;
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[0])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[1])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.F[2])));
            Bbox_3D->SetArrayField("F", F);

            TArray<TSharedPtr<FJsonValue>> G;
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[0])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[1])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.G[2])));
            Bbox_3D->SetArrayField("G", G);

            TArray<TSharedPtr<FJsonValue>> H;
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[0])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[1])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World.H[2])));
            Bbox_3D->SetArrayField("H", H);

            singleDetectionObj->SetObjectField("BoundingBox3D_World", Bbox_3D);

            TSharedPtr<FJsonObject> Bbox_3D_Raw = MakeShareable(new FJsonObject);
            A.Reset();
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[0])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[1])));
            A.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.A[2])));
            Bbox_3D_Raw->SetArrayField("A", A);

            B.Reset();
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[0])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[1])));
            B.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.B[2])));
            Bbox_3D_Raw->SetArrayField("B", B);

            C.Reset();
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[0])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[1])));
            C.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.C[2])));
            Bbox_3D_Raw->SetArrayField("C", C);

            D.Reset();
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[0])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[1])));
            D.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.D[2])));
            Bbox_3D_Raw->SetArrayField("D", D);

            E.Reset();
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[0])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[1])));
            E.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.E[2])));
            Bbox_3D_Raw->SetArrayField("E", E);

            F.Reset();
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[0])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[1])));
            F.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.F[2])));
            Bbox_3D_Raw->SetArrayField("F", F);

            G.Reset();
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[0])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[1])));
            G.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.G[2])));
            Bbox_3D_Raw->SetArrayField("G", G);

            H.Reset();
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[0])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[1])));
            H.Add(MakeShareable(new FJsonValueNumber(detection.BoundingBox3D_World_Raw.H[2])));
            Bbox_3D_Raw->SetArrayField("H", H);

            singleDetectionObj->SetObjectField("BoundingBox3D_World_Raw", Bbox_3D_Raw);

#endif          
            ////////////////////////////////////
            // SKELETON   //////////////////////
            ////////////////////////////////////

#if 0
            TArray<TSharedPtr<FJsonValue>> globalOrientation;
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.X)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.Y)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.Z)));
            globalOrientation.Add(MakeShareable(new FJsonValueNumber(detection.GlobalRootOrientation.W)));

            TSharedPtr<FJsonObject> LocalPositionPerJoint = MakeShareable(new FJsonObject);
            TSharedPtr<FJsonObject> LocalOrientationPerJoint = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.LocalPositionPerJoint.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> joint_position;
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].X)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].Y)));
                joint_position.Add(MakeShareable(new FJsonValueNumber(detection.LocalPositionPerJoint[i].Z)));

                LocalPositionPerJoint->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), joint_position);

                TArray<TSharedPtr<FJsonValue>> keypoints;
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].X)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Y)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Z)));

                Keypoints3D_34->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), keypoints);

                TArray<TSharedPtr<FJsonValue>> joint_orientation;
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].X)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].Y)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].Z)));
                joint_orientation.Add(MakeShareable(new FJsonValueNumber(detection.LocalOrientationPerJoint[i].W)));

                LocalOrientationPerJoint->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), joint_orientation);
            }

#endif
            TSharedPtr<FJsonObject> Keypoints3D_34 = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints3D_34.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> keypoints;
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].X)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Y)));
                keypoints.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D_34[i].Z)));

                Keypoints3D_34->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), keypoints);

            }

            TSharedPtr<FJsonObject> Keypoints3D = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints3D.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_3d;
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].X)));
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].Y)));
                kp_3d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints3D[i].Z)));

                Keypoints3D->SetArrayField(enumToString((BODY_PARTS_POSE_18)i), kp_3d);
            }

            TSharedPtr<FJsonObject> Keypoints2D_34 = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints2D_34.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_2d_34;
                kp_2d_34.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D_34[i].X)));
                kp_2d_34.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D_34[i].Y)));

                Keypoints2D_34->SetArrayField(enumToString((BODY_PARTS_POSE_34)i), kp_2d_34);
            }

            TSharedPtr<FJsonObject> Keypoints2D = MakeShareable(new FJsonObject);
            for (int i = 0; i < detection.Keypoints2D.Num(); i++) {

                TArray<TSharedPtr<FJsonValue>> kp_2d;
                kp_2d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D[i].X)));
                kp_2d.Add(MakeShareable(new FJsonValueNumber(detection.Keypoints2D[i].Y)));

                Keypoints2D->SetArrayField(enumToString((BODY_PARTS_POSE_18)i), kp_2d);
            }

            singleDetectionObj->SetObjectField("Keypoints3D_34", Keypoints3D_34);
            //singleDetectionObj->SetObjectField("Local_Position_Per_Joint", LocalPositionPerJoint);
            //singleDetectionObj->SetObjectField("Local_Orientation_Per_Joint", LocalOrientationPerJoint);
            //singleDetectionObj->SetArrayField("Global_Root_Orientation", globalOrientation);
            singleDetectionObj->SetObjectField("Keypoints3D", Keypoints3D);
            singleDetectionObj->SetObjectField("Keypoints2D", Keypoints2D);
            singleDetectionObj->SetObjectField("Keypoints2D_34", Keypoints2D_34);


            TSharedRef<FJsonValueObject> singleDetectionValue = MakeShareable(new FJsonValueObject(singleDetectionObj));
            ObjectDetections.Add(singleDetectionValue);
        }
        Detections->SetArrayField("ObjectDetections", ObjectDetections);
        frameObj->SetObjectField("Detections", Detections);

        TSharedRef<FJsonValueObject> frameValue = MakeShareable(new FJsonValueObject(frameObj));
        Frames.Add(frameValue);
    }

    JsonObject->SetArrayField("Frames", Frames);

    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
    FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

    return OutputString;
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////// Perf Structs ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

USTRUCT()
struct FPerfJsonMetaDataRunInfo
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FString CpuModel;

    UPROPERTY()
    int CudaVersion;

    UPROPERTY()
    FString Date;

    UPROPERTY()
    FString GpuModel;

    UPROPERTY()
    FString HostName;

    UPROPERTY()
    FString OSVersion;
};


USTRUCT()
struct FPerfJsonSequenceDepthParameters
{
    GENERATED_BODY()

public:

    UPROPERTY()
    ESlDepthMode DepthMode;
};

USTRUCT()
struct FPerfJsonSequencePositionalTrackingParameters
{
    GENERATED_BODY()

public:

    UPROPERTY()
        bool bEnableAreaMemory = false;

    UPROPERTY()
        bool bSetAsStatic = true;
};

USTRUCT()
struct FPerfJsonMetaData
{
    GENERATED_BODY()

public:
    UPROPERTY()
        FPerfJsonMetaDataRunInfo RunInfo;
};

USTRUCT()
struct FPerfJsonSequenceZEDParameters
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FSlBodyTrackingParameters BodyTrackingParameters;

    UPROPERTY()
    FSlBodyTrackingRuntimeParameters BodyTrackingRuntimeParameters;

    UPROPERTY()
    FPerfJsonSequenceDepthParameters DepthParameters;

    UPROPERTY()
    FPerfJsonSequencePositionalTrackingParameters PositionalTrackingParameters;
};

USTRUCT()
struct FPerfJsonSequenceParameters
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FPerfJsonSequenceZEDParameters ZEDParameters;
};

USTRUCT()
struct FPerfJsonSequencePerformanceResultsObjectDetectionRuntime
{
    GENERATED_BODY()

public:

    UPROPERTY()
    float RetrieveTime_MS;

    UPROPERTY()
    float GrabTime_MS;

    UPROPERTY()
    float AppFPS;
};

USTRUCT()
struct FPerfJsonSequencePerformanceResults
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FPerfJsonSequencePerformanceResultsObjectDetectionRuntime ObjectDetectionRunTime;
};

USTRUCT()
struct FPerfJsonSequence
{
    GENERATED_BODY()

public:

    UPROPERTY()
    bool IsFusion;

    UPROPERTY()
    FPerfJsonSequenceParameters Parameters;
    UPROPERTY()
    FPerfJsonSequencePerformanceResults PerformanceResults;
};

USTRUCT()
struct FPerfJsonData
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FPerfJsonMetaData MetaData;

    UPROPERTY()
    TArray<FPerfJsonSequence> Sequences;
};


static FString SerializePerfJson(FPerfJsonData data)
{
    FString OutputString;


    // create a Json object and add a string field
    TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);

    ////////////////////////////////////
    /// METADATA ///////////////////////
    ////////////////////////////////////    

    TSharedPtr<FJsonObject> metadata = MakeShareable(new FJsonObject);
    TSharedPtr<FJsonObject> metadata_run_info = MakeShareable(new FJsonObject);
    TSharedPtr<FJsonObject> metadata_software = MakeShareable(new FJsonObject);

    metadata_run_info->SetStringField("CPU", data.MetaData.RunInfo.CpuModel);
    metadata_run_info->SetNumberField("CUDA", data.MetaData.RunInfo.CudaVersion);
    metadata_run_info->SetStringField("Date", data.MetaData.RunInfo.Date);
    metadata_run_info->SetStringField("GPU", data.MetaData.RunInfo.GpuModel);
    metadata_run_info->SetStringField("Host", data.MetaData.RunInfo.HostName);
    metadata_run_info->SetStringField("OS", data.MetaData.RunInfo.OSVersion);

    metadata->SetObjectField("RUN_info", metadata_run_info);

    JsonObject->SetObjectField("Metadata", metadata);

    TArray<TSharedPtr<FJsonValue>> Sequences;
    for (auto sequence : data.Sequences) 
    {
        TSharedPtr<FJsonObject> sequenceObj = MakeShareable(new FJsonObject);

        sequenceObj->SetBoolField("IsFusion", sequence.IsFusion);

        TSharedPtr<FJsonObject> parametersObj = MakeShareable(new FJsonObject);
        TSharedPtr<FJsonObject> zedParametersObj = MakeShareable(new FJsonObject);

        TSharedPtr<FJsonObject> bodyTrackingParametersObj = MakeShareable(new FJsonObject);
        TSharedPtr<FJsonObject> depthParametersObj = MakeShareable(new FJsonObject);
    
        bodyTrackingParametersObj->SetStringField("body_format", EnumToString(sequence.Parameters.ZEDParameters.BodyTrackingParameters.BodyFormat));
        bodyTrackingParametersObj->SetNumberField("detection_confidence_threshold", sequence.Parameters.ZEDParameters.BodyTrackingRuntimeParameters.DetectionConfidenceThreshold);
        bodyTrackingParametersObj->SetStringField("detection_model", EnumToString(sequence.Parameters.ZEDParameters.BodyTrackingParameters.DetectionModel));
        bodyTrackingParametersObj->SetBoolField("enable_body_fitting", sequence.Parameters.ZEDParameters.BodyTrackingParameters.bEnableBodyFitting);
        bodyTrackingParametersObj->SetBoolField("enable_tracking", sequence.Parameters.ZEDParameters.BodyTrackingParameters.bEnableTracking);

        zedParametersObj->SetObjectField("BodyTrackingParameters", bodyTrackingParametersObj);

        depthParametersObj->SetStringField("depth_mode", EnumToString(sequence.Parameters.ZEDParameters.DepthParameters.DepthMode));
        zedParametersObj->SetObjectField("DepthParameters", depthParametersObj);

        parametersObj->SetObjectField("ZEDParameters", zedParametersObj);

        sequenceObj->SetObjectField("Parameters", parametersObj);

        TSharedPtr<FJsonObject> performanceResultsObj = MakeShareable(new FJsonObject);
        TSharedPtr<FJsonObject> objectDetectionRuntimeObj = MakeShareable(new FJsonObject);

        objectDetectionRuntimeObj->SetNumberField("RetrieveTime_MS", sequence.PerformanceResults.ObjectDetectionRunTime.RetrieveTime_MS);
        objectDetectionRuntimeObj->SetNumberField("GrabTime_MS", sequence.PerformanceResults.ObjectDetectionRunTime.GrabTime_MS);
        objectDetectionRuntimeObj->SetNumberField("AppFPS", sequence.PerformanceResults.ObjectDetectionRunTime.AppFPS);

        performanceResultsObj->SetObjectField("ObjectDetectionRunTime", objectDetectionRuntimeObj);
        sequenceObj->SetObjectField("PerformanceResults", performanceResultsObj);

        TSharedRef<FJsonValueObject> sequenceValue = MakeShareable(new FJsonValueObject(sequenceObj));
        Sequences.Add(sequenceValue);
    }

    JsonObject->SetArrayField("Sequences", Sequences);

    return OutputString;
}

FPerfJsonMetaData getMetaData() {
    int zed_major, zed_minor, zed_patch;
    getZEDSDKBuildVersion(zed_major, zed_minor, zed_patch);

    FPerfJsonMetaData metadata;

    // current date/time based on current system
    time_t now = time(0);
    std::string date(ctime(&now));
    if (date.back() == '\n') date.pop_back(); // remove EOL
    metadata.RunInfo.Date = FString(date.c_str());
    metadata.RunInfo.HostName = FString(getHostName().c_str());
    metadata.RunInfo.OSVersion = FString(getOS().c_str());
    metadata.RunInfo.CpuModel = FString(getCPU().c_str());
    auto GPU_infos = getGPU();
    metadata.RunInfo.GpuModel = FString(GPU_infos.first.c_str());
    metadata.RunInfo.CudaVersion = GPU_infos.second;

    return metadata;
}

static std::string getCPU() {
    std::string cpuName = "";
#ifdef _WIN32
    // Get extended ids.
    int CPUInfo[4] = { -1 };
    __cpuid(CPUInfo, 0x80000000);
    unsigned int nExIds = CPUInfo[0];
    // Get the information associated with each extended ID.
    char CPUBrandString[0x40] = { 0 };
    for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
        __cpuid(CPUInfo, i);
        // Interpret CPU brand string and cache information.
        if (i == 0x80000002) {
            memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
        }
        else if (i == 0x80000003) {
            memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
        }
        else if (i == 0x80000004) {
            memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
        }
    }
    std::string result(CPUBrandString);
    while (result.back() == ' ') result.pop_back(); // remove white spaces
    cpuName = result;
#else
    std::ifstream file("/proc/cpuinfo");
    std::string line;
    std::string result;
    if (file.is_open()) {
        while (file.good()) {
            getline(file, line);
            if (line.find("model name") != std::string::npos) {
                bool delimPass = false;
                bool lastSpace = false;
                for (int i = 0; i < line.size(); i++) {
                    if (delimPass) {
                        if (!(lastSpace && line.at(i) == ' '))
                            result += line.at(i);
                        if (line.at(i) == ' ')
                            lastSpace = true;
                        else
                            lastSpace = false;
                    }
                    if (!delimPass && line.at(i) == ':')
                        delimPass = true;
                }
                break;
            }
        }
    }
    file.close();
    cpuName = result;
#endif
    return cpuName;
}

static std::pair<std::string, int> getGPU() {
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    int cuda_v = 0;

    cudaRuntimeGetVersion(&cuda_v);
    int cuda_major = floor(cuda_v / 1000);
    int cuda_minor = floor((cuda_v - (cuda_major * 1000)) / 10);
    cuda_v = cuda_major * 10 + cuda_minor;

    return std::make_pair(std::string(prop.name), cuda_v);
}

static std::string getOS() {
    std::string osName = "";
    osName = "Windows 10";
    return osName;
}

static std::string getHostName() {
    std::string hostName;
#if _WIN32
#define INFO_BUFFER_SIZE 32767
    /*TCHAR infoBuf[INFO_BUFFER_SIZE];
    DWORD bufCharCount = INFO_BUFFER_SIZE;
    GetComputerName(infoBuf, &bufCharCount);*/

    char name[1024] = "";
    const int result = gethostname(name, sizeof name);
    hostName = std::string(name);
#else
    char infoBuf[HOST_NAME_MAX];
    gethostname(infoBuf, HOST_NAME_MAX);
    hostName = std::string(infoBuf);
#endif
    return hostName;
}
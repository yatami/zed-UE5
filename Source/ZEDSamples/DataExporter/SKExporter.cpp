// Fill out your copyright notice in the Description page of Project Settings.

#include "DataExporter/SKExporter.h"

// Sets default values
ASKExporter::ASKExporter()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
}

/*ASKExporter::~ASKExporter()
{
    is_recording_ = false;

    if (IsFileOpen())
    {
        CloseFile();
    }
}*/

// Called when the game starts or when spawned
void ASKExporter::BeginPlay()
{
	Super::BeginPlay();
	

    //GSlCameraProxy->OnCameraOpened.AddDynamic(this, &ASKExporter::Init);
    
    /*auto GrabDelegateHandle = GSlCameraProxy->AddToGrabDelegate([this](ESlErrorCode ErrorCode, const FSlTimestamp& Timestamp)
        {
            Init();
        });*/
}

bool ASKExporter::IsInit()
{
    return is_init_;
}

bool ASKExporter::IsFileOpen() const
{
    return log_file_handle_ != nullptr;
}

void ASKExporter::CloseFile()
{
    if (IsFileOpen())
        delete log_file_handle_;

    log_file_handle_ = nullptr;
}

void ASKExporter::CreateJSONFile(const std::string& file_path)
{
    try {
        CloseFile();

        IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
        log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));

    }
    catch (std::exception& ex) {
        UE_LOG(LogTemp, Error, TEXT("Can not create json file %s"), ex.what());
    }
}

void ASKExporter::WriteString(const std::string& str) const
{
    try 
    {
        if (log_file_handle_) 
        {
            FString line_f(str.c_str());
            log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
        }
        else
            UE_LOG(LogTemp, Error, TEXT("Attempt to write to recording log file when file was not opened"));
    }
    catch (std::exception& ex) 
    {
        UE_LOG(LogTemp, Error, TEXT("write to recording file failed %s"), ex.what());
    }
}


void ASKExporter::Init()
{
    if (IsInit()) return;
    std::string path = std::string(TCHAR_TO_UTF8(*JSONFilename));
    CreateJSONFile(path);

    data = FJsonDataSet();
    FJsonMetaData meta;

    if (GSlCameraProxy)
    {
        long long now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int64 sequence_id = IS_MICROSECONDS == true ? ( now_ns / 1000) : now_ns;

        FIntPoint res = FIntPoint::ZeroValue;
        int width = GSlCameraProxy->GetCameraInformation(res).Resolution.X;
        int height = GSlCameraProxy->GetCameraInformation(res).Resolution.Y;
        meta.ImageHeight = height;
        meta.ImageWidth = width;
        meta.TargetFPS = GSlCameraProxy->GetCameraFPS();
        meta.InvalidValue = GT_INVALID_VALUE;
        meta.IsMicroseconds = IS_MICROSECONDS;
        meta.ZEDSerialNumber = GSlCameraProxy->GetCameraInformation(res).SerialNumber;
        meta.Bbox3dMinimumVolume = BBOX_MINIMUM_VOLUME;
        meta.IsRealZED = false;
        meta.IsRectified = true;
        meta.SequenceID = sequence_id;


        __fx = GSlCameraProxy->GetCameraInformation(res).CalibrationParameters.LeftCameraParameters.HFocal;
        __fy = GSlCameraProxy->GetCameraInformation(res).CalibrationParameters.LeftCameraParameters.VFocal;
        __cx = GSlCameraProxy->GetCameraInformation(res).CalibrationParameters.LeftCameraParameters.OpticalCenterX;
        __cy = GSlCameraProxy->GetCameraInformation(res).CalibrationParameters.LeftCameraParameters.OpticalCenterY;
    }

    data.Metadata = meta;

    UE_LOG(LogTemp, Warning, TEXT("Is Init"));
    is_init_ = true;
}

// Called every frame
void ASKExporter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    if (GSlIsGrabThreadIdInitialized) Init();
}

void ASKExporter::PushNewFrame(int FrameCount, FSlTimestamp TS, TMap<int, USkeletalMeshComponent*> Avatars, FSlPose CameraPose)
{
    if (!IsInit()) return;

    FJsonFrameData fdata;

    uint64 ts = IS_MICROSECONDS ? (TS.timestamp.data_ns / 1000) : TS.timestamp;
    fdata.EpochTimeStamp = ts;

    std::ostringstream fileName;
    fileName // << "SBS_"
        << "_"
        << fdata.EpochTimeStamp
        << "_"
        << std::setw(5)
        << std::setfill('0')
        << FrameCount
        << ".png";
    fdata.ImageFileName = UTF8_TO_TCHAR(fileName.str().c_str());
    fdata.FrameIndex = FrameCount;

    FJsonFramePoseData trackedPoseData;
    trackedPoseData.WorldPose = FJsonMatrix4x4(CameraPose.Transform);

    fdata.TrackedPose = trackedPoseData;

    FJsonFrameDetections frameDetections;

    for (auto &it : Avatars)
    {
        FJsonSingleDetection singleDetection;
        singleDetection.ObjectID = it.Key;
        singleDetection.ObjectType = 0;

        FJsonSkeletonData skeletonData34;

        // Body 34
        for (int idx = 0; idx < (int)BODY_PARTS_POSE_34::LAST; idx++) {

            if (targetBone_34[idx] == "not_found") {

                skeletonData34.Keypoints3D_34.Add(FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE));
                skeletonData34.Keypoints2D_34.Add(FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE));

                continue;
            }

            FVector keypoint;
            if (idx == (int)BODY_PARTS_POSE_34::PELVIS) { // The PELIVS kp is not at the same position in the SDK and in UE. Create a Fake kp at the correct position.
                keypoint = (it.Value->GetBoneLocation(targetBone_34[(int)BODY_PARTS_POSE_34::LEFT_HIP], EBoneSpaces::WorldSpace) + it.Value->GetBoneLocation(targetBone_34[(int)BODY_PARTS_POSE_34::RIGHT_HIP], EBoneSpaces::WorldSpace)) / 2;
            }
            else {
                keypoint = it.Value->GetBoneLocation(targetBone_34[idx], EBoneSpaces::WorldSpace);
            }

            // if the joint does not exist on the model, ignore it
            if (keypoint == FVector::ZeroVector)
            {
                skeletonData34.Keypoints3D_34.Add(FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE));
            }
            else
            {
                skeletonData34.Keypoints3D_34.Add(convertFromUUToUnityCoordinateSystem(worldToCam(CameraPose.Transform, keypoint)));
            }

            FVector2D keypoint_2d = projectPoint_(convertFromUUToImageCoordinateSystem(worldToCam(CameraPose.Transform, keypoint)), __fx, __fy, __cx, __cy);
            if (keypoint_2d.ContainsNaN())
            {
                keypoint_2d = FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
            }
            skeletonData34.Keypoints2D_34.Add(keypoint_2d);
        }

        FJsonSkeletonData skeletonData18;
        
        // Keypoints 18
        for (int idx = 0; idx < (int)BODY_PARTS_POSE_18::LAST; idx++) {
            if (targetBone_18[idx] == "not_found") {

                skeletonData18.Keypoints3D.Add(FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE));
                skeletonData18.Keypoints2D.Add(FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE));
                continue;
            }

            FVector keypoint = it.Value->GetBoneLocation(targetBone_18[idx], EBoneSpaces::WorldSpace);

            // if the joint does not exist on the model, ignore it
            if (keypoint == FVector::ZeroVector)
            {
                skeletonData18.Keypoints3D.Add(FVector(GT_INVALID_VALUE, GT_INVALID_VALUE, GT_INVALID_VALUE));

            }
            else
            {
                skeletonData18.Keypoints3D.Add(convertFromUUToUnityCoordinateSystem(worldToCam(CameraPose.Transform, keypoint)));
            }

            FVector2D keypoint_2d = projectPoint_(convertFromUUToImageCoordinateSystem(worldToCam(CameraPose.Transform, keypoint)), __fx, __fy, __cx, __cy);
            if (keypoint_2d.ContainsNaN())
            {
                keypoint_2d = FVector2D(GT_INVALID_VALUE, GT_INVALID_VALUE);
            }
            skeletonData18.Keypoints2D.Add(keypoint_2d);      
        }

        singleDetection.Keypoints2D = skeletonData18.Keypoints2D;
        singleDetection.Keypoints2D_34 = skeletonData34.Keypoints2D_34;
        singleDetection.Keypoints3D = skeletonData18.Keypoints3D;
        singleDetection.Keypoints3D_34 = skeletonData34.Keypoints3D_34;

        frameDetections.ObjectDetections.Push(singleDetection);
    }

    fdata.Detections = frameDetections;

    data.Frames.Add(fdata);
}

void ASKExporter::Save()
{
    if (!IsFileOpen()) {
        UE_LOG(LogTemp, Warning, TEXT("File was not open"));
    }
    else {
        WriteString(std::string(TCHAR_TO_UTF8(*SerializeJson(data))));

        CloseFile();
        UE_LOG(LogTemp, Warning,TEXT("Recording: Stopped"));
    }

    UE_LOG(LogTemp, Warning,TEXT("Data saved to: %s"), *JSONFilename);
}

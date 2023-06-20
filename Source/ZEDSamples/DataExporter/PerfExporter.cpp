// Fill out your copyright notice in the Description page of Project Settings.

#include "DataExporter/PerfExporter.h"

// Sets default values
APerfExporter::APerfExporter()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
}

// Called when the game starts or when spawned
void APerfExporter::BeginPlay()
{
	Super::BeginPlay();
	

    //GSlCameraProxy->OnCameraOpened.AddDynamic(this, &APerfExporter::Init);
    
    /*auto GrabDelegateHandle = GSlCameraProxy->AddToGrabDelegate([this](ESlErrorCode ErrorCode, const FSlTimestamp& Timestamp)
        {
            Init();
        });*/
}

bool APerfExporter::IsInit()
{
    return is_init_;
}

bool APerfExporter::IsFileOpen() const
{
    return log_file_handle_ != nullptr;
}

void APerfExporter::CloseFile()
{
    if (IsFileOpen())
        delete log_file_handle_;

    log_file_handle_ = nullptr;
}

void APerfExporter::CreateJSONFile(const std::string& file_path)
{
    CloseFile();

    IPlatformFile& platform_file = FPlatformFileManager::Get().GetPlatformFile();
    log_file_handle_ = platform_file.OpenWrite(*FString(file_path.c_str()));
}

void APerfExporter::WriteString(const std::string& str) const
{

    if (log_file_handle_) 
    {
        FString line_f(str.c_str());
        log_file_handle_->Write((const uint8*)TCHAR_TO_ANSI(*line_f), line_f.Len());
    }
    else
        UE_LOG(LogTemp, Error, TEXT("Attempt to write to recording log file when file was not opened"));

}


void APerfExporter::Init()
{
    if (IsInit()) return;
    std::string path = std::string(TCHAR_TO_UTF8(*JSONFilename));
    CreateJSONFile(path);

    data = FPerfJsonData();
    

    UE_LOG(LogTemp, Warning, TEXT("Is Init"));
    is_init_ = true;
}

// Called every frame
void APerfExporter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    if (IsInit() && current_frame == GSlCameraProxy->GetSVONumberOfFrames() - 2)
    {
        Save();
    }

    if (GSlIsGrabThreadIdInitialized) Init();
}

void APerfExporter::PushNewFrame(int FrameCount, FSlTimestamp TS, TMap<int, USkeletalMeshComponent*> Avatars, FSlPose CameraPose)
{
    if (!IsInit()) return;


}

void APerfExporter::Save()
{
    if (!IsFileOpen()) {
        UE_LOG(LogTemp, Warning, TEXT("File was not open"));
    }
    else {
        WriteString(std::string(TCHAR_TO_UTF8(*SerializePerfJson(data))));

        CloseFile();
        UE_LOG(LogTemp, Warning, TEXT("Data saved to: %s"), *JSONFilename);
    }

}

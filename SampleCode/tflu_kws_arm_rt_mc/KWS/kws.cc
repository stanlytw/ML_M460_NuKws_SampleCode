/*
 * Copyright (C) 2021 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Keyword spotting example code using MFCC feature extraction
 * and neural network.
 */

#include "kws.h"
#include <float.h>

KWS::KWS(int slidingWinLen, uint32_t * audio_buffer)
{
    if (this->_InitModel()) {
			  this->audioBuffer = audio_buffer;
        this->slidingWindowLen = slidingWinLen;
        this->InitKws();
    }
}

//KWS::KWS(int16_t * ptrAudioBuffer, uint32_t nElements)
//{
//    if (this->_InitModel()) {
//        this->audioBuffer = std::move(std::vector<int16_t>(
//                                            ptrAudioBuffer,
//                                            ptrAudioBuffer + nElements));
//        this->recordingWin = model->GetNumFrames();
//        this->slidingWindowLen = 1;
//        this->InitKws();
//    }
//}

void KWS::InitKws()
{
    if (!model->IsInited()) {
        printf("Warning: model has not been initialised\r\n");
        model->Init();
    }
    
		numInputDims[0] = model->GetInputShape()->data[0]; //Check input data  1x128x3 
    numInputDims[1] = model->GetInputShape()->data[1];
    numInputDims[2] = model->GetInputShape()->data[2];
    numInputDims[3] = model->GetInputShape()->data[3];
    numOutClasses = model->GetOutputShape()->data[1];  // Output shape should be [1, numOutClasses].
		
		 // Following are for debug purposes.
    printf("Initialising VWW model..\r\n");
    printf("numInputDims: %d-%d-%d-%d\r\n", numInputDims[0], numInputDims[1], numInputDims[2], numInputDims[3]);
    printf("numOutClasses: %d\r\n", numOutClasses);
		
    numMfccFeatures = model->GetNumMfccFeatures();
    numFrames = model->GetNumFrames();
    frameLen = model->GetFrameLen();
    frameShift = model->GetFrameShift();
    numOutClasses = model->GetOutputShape()->data[1];  // Output shape should be [1, numOutClasses].
		recordingWin = numFrames; // In this rt_mc, the recording window is 1s, so it is same as number of per second frames.

    // Following are for debug purposes.
    printf("Initialising KWS object..\r\n");
    printf("numMfccFeatures: %d\r\n", numMfccFeatures);
    printf("numFrames: %d\r\n", numFrames);
    printf("frameLen: %d\r\n", frameLen);
    printf("frameShift: %d\r\n", frameShift);
    printf("numOutClasses: %d\r\n", numOutClasses);
		printf("recordingWin: %d\r\n", recordingWin);

    //mfcc =  std::unique_ptr<MFCC>(new MFCC(numMfccFeatures, frameLen));
		mfcc =  new MFCC(numMfccFeatures, frameLen);
    mfccBuffer = std::vector<float>(numFrames * numMfccFeatures, 0.0);
    output = std::vector<float>(numOutClasses, 0.0);
    averagedOutput = std::vector<float>(numOutClasses, 0.0);
    predictions = std::vector<float>(slidingWindowLen * numOutClasses, 0.0);
    audioBlockSize = recordingWin * frameShift;
    audioBufferSize = audioBlockSize + frameLen - frameShift;
}

void KWS::ExtractFeatures()
{
	//transfer from uint32 to int16
	for(uint16_t i = 0; i < 8000; i++)
    {
        audio_buffer_16bit[2*i] = audioBuffer[i]&0x0000FFFF;
			  audio_buffer_16bit[2*i+1] = (audioBuffer[i]>>16)&0x0000FFFF;
			  
    }
//		for(uint16_t i = 4000; i < 6000; i++)
//    {
//        printf("%d, %d \n", 2*i, audio_buffer_16bit[2*i]);
//			  printf("%d, %d \n", 2*i+1, audio_buffer_16bit[2*i+1]);
//    }
	
    if (numFrames > recordingWin) {
        // Move old features left.
        memmove(mfccBuffer.data(),
                mfccBuffer.data() + (recordingWin * numMfccFeatures),
                (numFrames - recordingWin) * numMfccFeatures * sizeof(float));
    }
    // Compute features only for the newly recorded audio.
    int32_t mfccBufferHead = (numFrames - recordingWin) * numMfccFeatures;
    for (uint16_t f = 0; f < recordingWin; f++) {
			  mfcc->MfccCompute(audio_buffer_16bit + (f * frameShift), &mfccBuffer[mfccBufferHead]);
        mfccBufferHead += numMfccFeatures;
    }
}

void KWS::ExtractFeatures_overlap(uint8_t count)
{
	//transfer from uint32 to int16
	for(int i = 0; i < 640; i++)
    {
        audio_buffer_16bit[i] = audioBuffer[i];
    }
		
		//The first framelen no need replace
		if(count != 0){
		//	  // process the overlap bcs the frameShift < frameLen
		    memcpy(&audio_buffer_16bit[0], &audio_half, 320 * sizeof(int16_t));
		}
	
    if (numFrames > recordingWin) {
        // Move old features left.
        memmove(mfccBuffer.data(),
                mfccBuffer.data() + (recordingWin * numMfccFeatures),
                (numFrames - recordingWin) * numMfccFeatures * sizeof(float));
    }
    // Compute features only for the newly recorded audio.
    int32_t mfccBufferHead = (numFrames - recordingWin) * numMfccFeatures;
    for (uint16_t f = 0; f < recordingWin; f++) {
        //mfcc->MfccCompute(audioBuffer + (f * frameShift), &mfccBuffer[mfccBufferHead]);
			  mfcc->MfccCompute(audio_buffer_16bit + (f * frameShift), &mfccBuffer[mfccBufferHead]);
			  //mfcc->MfccCompute(audioBuffer.data() + (f * frameShift), &mfccBuffer[mfccBufferHead]);
			  //printf("audioBuffer: %d , %d\n", (audioBuffer[639]), (audioBuffer[0]));
        mfccBufferHead += numMfccFeatures;
    }
		
		//copy the old half data for next time half
		memcpy(&audio_half, &audio_buffer_16bit[320], 320 * sizeof(int16_t));
}

void KWS::Classify()
{
    // Copy mfcc features into the TfLite tensor.
    float* inTensorData = tflite::GetTensorData<float>(model->GetInputTensor());
    memcpy(inTensorData, mfccBuffer.data(), numFrames * numMfccFeatures * sizeof(float));

    // Run inference on this data.
    model->RunInference();

    // Get output from the TfLite tensor.
    float* outTensorData = tflite::GetTensorData<float>(model->GetOutputTensor());
    memcpy(output.data(), outTensorData, numOutClasses * sizeof(float));
}

int KWS::GetTopClass(const std::vector<float>& prediction)
{
    int maxInd = 0;
    float maxVal = FLT_MIN;
    for (int i = 0; i < numOutClasses; i++) {
        if (maxVal < prediction[i]) {
            maxVal = prediction[i];
            maxInd = i;
        }
    }
    return maxInd;
}

void KWS::AveragePredictions()
{
    // Shift the old predictions left.
    memmove(predictions.data(),
            predictions.data() + numOutClasses,
            (slidingWindowLen - 1) * numOutClasses * sizeof(float));

    // Add new predictions at the end.
    memmove((predictions.data() + (slidingWindowLen - 1 ) * numOutClasses),
            output.data(),
            numOutClasses * sizeof(float));

    // Compute averages.
    float sum;
    for (int j = 0; j < numOutClasses; j++) {
        sum = 0;
        for(int i = 0; i < slidingWindowLen; i++) {
            sum += predictions[i*numOutClasses + j];
        }
        averagedOutput[j] = (sum / slidingWindowLen);
    }
}

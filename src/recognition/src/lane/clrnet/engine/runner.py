import cv2
import torch
import numpy as np
import random
import time
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

#from inverse_birdeyeview import IBE
from clrnet.models.registry import build_net
from clrnet.utils.net_utils import load_network
from mmcv.parallel import MMDataParallel
#from get_curve import curve, get_steer, draw_lanes

#from configs.clrnet.clr_resnet34_tusimple import img_h, img_w, cut_height
from configs.clrnet.clr_resnet34_tusimple import img_h, img_w, cut_height

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))))

device = 'cuda' if torch.cuda.is_available() else 'cpu'

class Runner(object):
    def __init__(self, cfg):
        torch.manual_seed(cfg.seed)
        np.random.seed(cfg.seed)
        random.seed(cfg.seed)
        self.cfg = cfg
        self.net = build_net(self.cfg)
        self.net = MMDataParallel(self.net, device_ids=range(self.cfg.gpus)).cuda()
        self.resume()
        self.steer = 0
        self.previous = 0 # 이전 steer 값

        self.plot = [] # steer 그림을 그리기 위한 plot list
        
        #print("device: ", device)


    def resume(self):
        if not self.cfg.load_from and not self.cfg.finetune_from:
            return
        load_network(self.net, self.cfg.load_from, finetune_from=self.cfg.finetune_from)
        

    def detect(self, ori_img): # test만 실행

        data = np.array(ori_img) # 받아온 이미지
        data = data[cut_height:, :, :] # cut_height
        data = cv2.resize(data, (img_w, img_h), interpolation=cv2.INTER_CUBIC) # resize(img_h, img_w)
        
        data = data.astype(np.float32) / 255.0 # normalize
        data = to_tensor(data) # tensor
        data = data.permute(2, 0, 1) # BGR -> RGB
        data = data.unsqueeze(dim = 0) # Batch size 지정
        data = data.to(device) # cuda로 올리기
        
        self.net.eval()

        with torch.no_grad():
            output = self.net(data) # 에측
            output = self.net.module.heads.get_lanes(output) # 예측
            lanes_xys = get_lane_coords(ori_img, output, cut_height) # 시각화

        return lanes_xys



def get_lane_coords(img, prediction, cut_height, width=4):
    img = np.array(img)

    for lanes in prediction:
        #print(type(lanes))
        #print(type(lanes[0]))
        lanes = [lane.to_array(img.shape[1], img.shape[0]) for lane in lanes] # width, height        

    lanes_xys = [] #초기화
    for _, lane in enumerate(lanes):
        xys = []
        for x, y in lane:
            if x <= 0 or y <= 0:
                continue
            x, y = int(x), int(y)
            xys.append((x, y))
        lanes_xys.append(xys)

    return lanes_xys



def to_tensor(data):

    if isinstance(data, torch.Tensor):
        return data
    elif isinstance(data, np.ndarray):
        return torch.from_numpy(data).float()
    elif isinstance(data, int):
        return torch.LongTensor([data])
    elif isinstance(data, float):
        return torch.FloatTensor([data])
    else:
        raise TypeError(f'type {type(data)} cannot be converted to tensor.')
    
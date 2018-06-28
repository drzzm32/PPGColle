package cn.ac.nya.ppgcalc.launcher;

import cn.ac.nya.ppgcalc.Breath;
import cn.ac.nya.ppgcalc.PPGData;
import cn.ac.nya.ppgcalc.SPO2;
import cn.ac.nya.ppgcalc.math.Transform;

public class Main {

    public static void main(String[] args) {
        PPGData data = Util.fromFile(1024, "E:\\me-new.txt");

        if (data != null) {
            Transform fft = new Transform(1024, 25.0);
            Transform.WaveData waveData;

            waveData = fft.output(fft.fft(data.getIR()), 0.5, 3.0);
            System.out.print("Heart rate: ");
            System.out.println(waveData.f * 60);

            Breath breath = new Breath(data.getIRInt());
            waveData = fft.output(fft.fft(breath.makeHs()), 0.2, 0.5);
            System.out.print("Breath rate: ");
            System.out.println(waveData.f * 60);

            SPO2 spo2 = new SPO2(0.000454, 0.983560);
            double sp = spo2.calcSpO2(data);
            System.out.print("SpO2: ");
            System.out.println(sp * 100);

        }

        System.out.println("done.");
    }
}

using System;
using System.IO;
using System.IO.Ports;
using System.Windows.Forms;
using System.Collections.Generic;

namespace PPGColle_CollectorHost
{
    public partial class MainForm : Form
    {
        struct PPGData
        {
            public int red;
            public int ir;

            public PPGData(int red, int ir)
            {
                this.red = red;
                this.ir = ir;
            }
        }

        string[] ports;
        LinkedList<PPGData> buffer = new LinkedList<PPGData>();
        bool state = false;

        public MainForm()
        {
            InitializeComponent();
        }

        private void boxSerial_Click(object sender, EventArgs e)
        {
            ports = SerialPort.GetPortNames();
            boxSerial.Items.Clear();
            foreach (string s in ports)
                boxSerial.Items.Add(s);
        }

        private void btnSerial_Click(object sender, EventArgs e)
        {
            if (boxSerial.SelectedIndex < 0) return;
            if (btnSerial.Text == "Connect")
            {
                btnSerial.Text = "Disconnect";
                serial.PortName = ports[boxSerial.SelectedIndex];
                if (!serial.IsOpen) serial.Open();
                boxSerial.Enabled = false;

                boxInvert.Enabled = true;
                btnReset.Enabled = true;
                btnData.Enabled = true;
            }
            else if (btnSerial.Text == "Disconnect")
            {
                btnSerial.Text = "Connect";
                if (serial.IsOpen) serial.Close();
                boxSerial.Enabled = true;

                boxInvert.Enabled = false;
                btnReset.Enabled = false;
                btnData.Enabled = false;
                btnSave.Enabled = false;
            }
        }

        private void serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (state)
            {
                string buf = serial.ReadLine();
                if (buf.Split(',').Length != 2) return;
                string[] data = buf.Split(',');

                int red = 0, ir = 0;
                if (!int.TryParse(data[0], out red))
                    red = -1;
                boxRed.Invoke(new MethodInvoker(() => boxRed.Text = red.ToString()));
                if (!int.TryParse(data[1], out ir))
                    ir = -1;
                boxIR.Invoke(new MethodInvoker(() => boxIR.Text = ir.ToString()));

                buffer.AddLast(new PPGData(red, ir));

                boxSum.Invoke(new MethodInvoker(() => boxSum.Text = buffer.Count.ToString()));
            }
            else
            {
                serial.ReadExisting();
            }
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            boxRed.Clear(); boxIR.Clear();
            boxSum.Text = "0";
            buffer.Clear();
        }

        private void btnData_Click(object sender, EventArgs e)
        {
            if (serial.IsOpen)
            {
                if (!state)
                {
                    state = true;
                    btnData.Text = "STOP";
                    btnSave.Enabled = false;
                }
                else
                {
                    state = false;
                    btnData.Text = "START";
                    btnSave.Enabled = true;
                }
            }
        }

        private void btnSave_Click(object sender, EventArgs e)
        {
            if (buffer.Count > 0 && !state) saveDialog.ShowDialog();
        }

        private void saveDialog_FileOk(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StreamWriter writer = new StreamWriter(saveDialog.FileName);

            if (boxInvert.Checked) Invert();

            foreach (PPGData data in buffer)
            {
                writer.WriteLine(data.red + ", " + data.ir);
            }

            writer.Flush();
            writer.Close();
            writer.Dispose();
            buffer.Clear();
            state = false;
        }

        private void Invert()
        {
            int maxRed = 0, maxIR = 0;
            LinkedList<PPGData> copy = new LinkedList<PPGData>(buffer);

            foreach (PPGData data in buffer)
            {
                if (data.red > maxRed) maxRed = data.red;
                if (data.ir > maxIR) maxIR = data.ir;
            }

            buffer.Clear();
            foreach (PPGData data in copy)
            {
                buffer.AddLast(new PPGData(maxRed - data.red, maxIR - data.ir));
            }
        }
    }
}

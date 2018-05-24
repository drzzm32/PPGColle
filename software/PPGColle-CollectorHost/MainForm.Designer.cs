namespace PPGColle_CollectorHost
{
    partial class MainForm
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.groupSerial = new System.Windows.Forms.GroupBox();
            this.btnSerial = new System.Windows.Forms.Button();
            this.boxSerial = new System.Windows.Forms.ComboBox();
            this.groupData = new System.Windows.Forms.GroupBox();
            this.btnData = new System.Windows.Forms.Button();
            this.btnSave = new System.Windows.Forms.Button();
            this.boxInvert = new System.Windows.Forms.CheckBox();
            this.btnReset = new System.Windows.Forms.Button();
            this.boxSum = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.boxIR = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.boxRed = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.saveDialog = new System.Windows.Forms.SaveFileDialog();
            this.serial = new System.IO.Ports.SerialPort(this.components);
            this.groupSerial.SuspendLayout();
            this.groupData.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupSerial
            // 
            this.groupSerial.Controls.Add(this.btnSerial);
            this.groupSerial.Controls.Add(this.boxSerial);
            this.groupSerial.Location = new System.Drawing.Point(12, 12);
            this.groupSerial.Name = "groupSerial";
            this.groupSerial.Size = new System.Drawing.Size(216, 47);
            this.groupSerial.TabIndex = 0;
            this.groupSerial.TabStop = false;
            this.groupSerial.Text = "Serial config";
            // 
            // btnSerial
            // 
            this.btnSerial.Location = new System.Drawing.Point(135, 18);
            this.btnSerial.Name = "btnSerial";
            this.btnSerial.Size = new System.Drawing.Size(75, 23);
            this.btnSerial.TabIndex = 1;
            this.btnSerial.Text = "Connect";
            this.btnSerial.UseVisualStyleBackColor = true;
            this.btnSerial.Click += new System.EventHandler(this.btnSerial_Click);
            // 
            // boxSerial
            // 
            this.boxSerial.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.boxSerial.FormattingEnabled = true;
            this.boxSerial.Location = new System.Drawing.Point(6, 20);
            this.boxSerial.Name = "boxSerial";
            this.boxSerial.Size = new System.Drawing.Size(123, 20);
            this.boxSerial.TabIndex = 0;
            this.boxSerial.Click += new System.EventHandler(this.boxSerial_Click);
            // 
            // groupData
            // 
            this.groupData.Controls.Add(this.btnData);
            this.groupData.Controls.Add(this.btnSave);
            this.groupData.Controls.Add(this.boxInvert);
            this.groupData.Controls.Add(this.btnReset);
            this.groupData.Controls.Add(this.boxSum);
            this.groupData.Controls.Add(this.label3);
            this.groupData.Controls.Add(this.boxIR);
            this.groupData.Controls.Add(this.label2);
            this.groupData.Controls.Add(this.boxRed);
            this.groupData.Controls.Add(this.label1);
            this.groupData.Location = new System.Drawing.Point(12, 65);
            this.groupData.Name = "groupData";
            this.groupData.Size = new System.Drawing.Size(216, 140);
            this.groupData.TabIndex = 1;
            this.groupData.TabStop = false;
            this.groupData.Text = "Data store";
            // 
            // btnData
            // 
            this.btnData.Enabled = false;
            this.btnData.Font = new System.Drawing.Font("宋体", 9F);
            this.btnData.Location = new System.Drawing.Point(6, 88);
            this.btnData.Name = "btnData";
            this.btnData.Size = new System.Drawing.Size(122, 46);
            this.btnData.TabIndex = 9;
            this.btnData.Text = "START";
            this.btnData.UseVisualStyleBackColor = true;
            this.btnData.Click += new System.EventHandler(this.btnData_Click);
            // 
            // btnSave
            // 
            this.btnSave.Enabled = false;
            this.btnSave.Location = new System.Drawing.Point(134, 88);
            this.btnSave.Name = "btnSave";
            this.btnSave.Size = new System.Drawing.Size(76, 46);
            this.btnSave.TabIndex = 8;
            this.btnSave.Text = "SAVE";
            this.btnSave.UseVisualStyleBackColor = true;
            this.btnSave.Click += new System.EventHandler(this.btnSave_Click);
            // 
            // boxInvert
            // 
            this.boxInvert.AutoSize = true;
            this.boxInvert.Checked = true;
            this.boxInvert.CheckState = System.Windows.Forms.CheckState.Checked;
            this.boxInvert.Enabled = false;
            this.boxInvert.Location = new System.Drawing.Point(8, 63);
            this.boxInvert.Name = "boxInvert";
            this.boxInvert.Size = new System.Drawing.Size(90, 16);
            this.boxInvert.TabIndex = 7;
            this.boxInvert.Text = "Invert data";
            this.boxInvert.UseVisualStyleBackColor = true;
            // 
            // btnReset
            // 
            this.btnReset.Enabled = false;
            this.btnReset.Location = new System.Drawing.Point(135, 59);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(75, 23);
            this.btnReset.TabIndex = 6;
            this.btnReset.Text = "Reset";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // boxSum
            // 
            this.boxSum.Location = new System.Drawing.Point(134, 32);
            this.boxSum.Name = "boxSum";
            this.boxSum.Size = new System.Drawing.Size(76, 21);
            this.boxSum.TabIndex = 5;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(146, 17);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 12);
            this.label3.TabIndex = 4;
            this.label3.Text = "Data sum";
            // 
            // boxIR
            // 
            this.boxIR.Location = new System.Drawing.Point(70, 32);
            this.boxIR.Name = "boxIR";
            this.boxIR.Size = new System.Drawing.Size(58, 21);
            this.boxIR.TabIndex = 3;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(73, 17);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 12);
            this.label2.TabIndex = 2;
            this.label2.Text = "IR value";
            // 
            // boxRed
            // 
            this.boxRed.Location = new System.Drawing.Point(6, 32);
            this.boxRed.Name = "boxRed";
            this.boxRed.Size = new System.Drawing.Size(58, 21);
            this.boxRed.TabIndex = 1;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 17);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(59, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "Red value";
            // 
            // saveDialog
            // 
            this.saveDialog.Filter = "Text file|*.txt";
            this.saveDialog.Title = "Save data...";
            this.saveDialog.FileOk += new System.ComponentModel.CancelEventHandler(this.saveDialog_FileOk);
            // 
            // serial
            // 
            this.serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serial_DataReceived);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(240, 217);
            this.Controls.Add(this.groupData);
            this.Controls.Add(this.groupSerial);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.MaximizeBox = false;
            this.MaximumSize = new System.Drawing.Size(256, 256);
            this.MinimumSize = new System.Drawing.Size(256, 256);
            this.Name = "MainForm";
            this.Text = "PPGColle-CollectorHost";
            this.groupSerial.ResumeLayout(false);
            this.groupData.ResumeLayout(false);
            this.groupData.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox groupSerial;
        private System.Windows.Forms.Button btnSerial;
        private System.Windows.Forms.ComboBox boxSerial;
        private System.Windows.Forms.GroupBox groupData;
        private System.Windows.Forms.TextBox boxSum;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox boxIR;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox boxRed;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.CheckBox boxInvert;
        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Button btnData;
        private System.Windows.Forms.Button btnSave;
        private System.Windows.Forms.SaveFileDialog saveDialog;
        private System.IO.Ports.SerialPort serial;
    }
}


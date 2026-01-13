import React, { useEffect, useRef } from 'react';
import io from 'socket.io-client';

const Visualizer = () => {
    const canvasRef = useRef(null);

    useEffect(() => {
        const socket = io('http://localhost:8082');
        const canvas = canvasRef.current;
        const ctx = canvas.getContext('2d');

        socket.on('connect', () => {
            console.log('Connected to WebSocket');
        });

        socket.on('Video', (data) => {
            const image = new Image();
            image.src = `data:image/jpeg;base64,${data}`;
            image.onload = () => {
                canvas.width = image.width;
                canvas.height = image.height;
                ctx.drawImage(image, 0, 0);
            };
        });

        socket.on('Objects', (data) => {
            try {
                const objects = JSON.parse(data);
                drawDetections(ctx, objects);
            } catch (error) {
                console.error("Error parsing Objects data:", error);
            }
        });

        const drawDetections = (ctx, objects) => {
            // Clear previous detections
            // ctx.clearRect(0, 0, canvas.width, canvas.height); // This will flicker, handled by redrawing image

            objects.forEach((obj) => {
                const { xmin, ymin, xmax, ymax, label_name } = obj;
                const color = 'red';
                const label = label_name || 'detection';

                // Scale coordinates if necessary (assuming they are normalized)
                const rectX = xmin * canvas.width;
                const rectY = ymin * canvas.height;
                const rectWidth = (xmax - xmin) * canvas.width;
                const rectHeight = (ymax - ymin) * canvas.height;

                // Draw bounding box
                ctx.strokeStyle = color;
                ctx.lineWidth = 2;
                ctx.strokeRect(rectX, rectY, rectWidth, rectHeight);

                // Draw label
                ctx.fillStyle = color;
                ctx.font = '16px Arial';
                const labelText = `${label}`;
                const textMetrics = ctx.measureText(labelText);
                ctx.fillRect(rectX, rectY - 20, textMetrics.width + 4, 20);
                ctx.fillStyle = 'white';
                ctx.fillText(labelText, rectX + 2, rectY - 5);
            });
        };

        return () => {
            socket.disconnect();
        };
    }, []);

    return (
        <div>
            <canvas ref={canvasRef} style={{ maxWidth: '100%' }} />
        </div>
    );
};

export default Visualizer;
